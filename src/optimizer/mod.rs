use crate::config::*;
use crate::consts::LBF_SAMPLE_CONFIG;
use crate::optimizer::compress::compression_phase;
use crate::optimizer::explore::exploration_phase;
use crate::optimizer::lbf::LBFBuilder;
use crate::optimizer::separator::Separator;
use crate::util::listener::{OptimizationPhase, ReportType, SolutionListener};
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use log::info;
use rand::rngs::Xoshiro256PlusPlus;
use rand::{Rng, SeedableRng};
use std::time::Duration;

pub mod compress;
pub mod explore;
pub mod lbf;
pub mod separator;
mod width_bound;
mod worker;

///Algorithm 11 from https://doi.org/10.48550/arXiv.2509.13329
/// 未得到完整、可行的探索候选时返回 None；不能把不合格的初始布局用于压缩或最终回退。
pub fn optimize(
    instance: SPInstance,
    rng: Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    initial_solution: Option<&SPSolution>,
) -> Option<SPSolution> {
    optimize_constrained(
        instance,
        rng,
        sol_listener,
        terminator,
        expl_config,
        cmpr_config,
        initial_solution,
        crate::constraints::PlacementConstraints::default(),
    )
    .ok()
    .flatten()
}

/// 对每个 demand=1 的固定 Item 保持全阶段姿态，定义或热启动冲突返回受控错误。
pub fn optimize_constrained(
    instance: SPInstance,
    mut rng: Xoshiro256PlusPlus,
    sol_listener: &mut impl SolutionListener,
    terminator: &mut impl Terminator,
    expl_config: &ExplorationConfig,
    cmpr_config: &CompressionConfig,
    initial_solution: Option<&SPSolution>,
    constraints: crate::constraints::PlacementConstraints,
) -> Result<Option<SPSolution>, crate::constraints::ConstraintError> {
    constraints.validate_instance(&instance)?;
    if let Some(initial) = initial_solution.filter(|_| !constraints.is_empty()) {
        if !constraints.matches_solution(initial) {
            let id = constraints
                .placements()
                .find(|fixed| {
                    initial
                        .layout_snapshot
                        .placed_items
                        .values()
                        .filter(|pi| pi.item_id == fixed.item_id && pi.d_transf == fixed.pose)
                        .count()
                        != 1
                })
                .map_or(0, |fixed| fixed.item_id);
            return Err(crate::constraints::ConstraintError::MissingOrChangedPose(
                id,
            ));
        }
        for placed in initial.layout_snapshot.placed_items.values() {
            if instance.items.get(placed.item_id).is_none() {
                return Err(crate::constraints::ConstraintError::UnknownItem(
                    placed.item_id,
                ));
            }
        }
    }
    let mut next_rng = || Xoshiro256PlusPlus::seed_from_u64(rng.next_u64());

    // Initialization belongs to the exploration phase and must consume the
    // same per-call budget as the separator that follows it.
    sol_listener.report_phase(OptimizationPhase::Exploration);
    terminator.new_timeout(expl_config.time_limit);

    // First build an initial solution if none is provided
    let has_initial_solution = initial_solution.is_some();
    let start_prob = match initial_solution {
        None => {
            let builder = LBFBuilder::new(instance.clone(), next_rng(), LBF_SAMPLE_CONFIG)
                .with_constraints(constraints.clone())?
                .construct_with_listener(terminator, sol_listener);
            match builder {
                Ok(builder) => builder.prob,
                Err(_) => return Ok(None),
            }
        }
        Some(init_sol) => {
            info!("[OPT] warm starting from provided initial solution");
            let mut prob = jagua_rs::probs::spp::entities::SPProblem::new(instance.clone());
            prob.restore(init_sol);
            prob
        }
    };

    // Begin by executing the exploration phase
    let mut expl_separator = Separator::new_constrained(
        instance.clone(),
        start_prob,
        next_rng(),
        expl_config.separator_config,
        constraints.clone(),
    )?;
    let solutions = exploration_phase(
        &instance,
        &mut expl_separator,
        sol_listener,
        terminator,
        expl_config,
        has_initial_solution,
    );
    let Some(final_explore_sol) = solutions.last().cloned() else {
        return Ok(None);
    };

    // Start the compression phase from the final solution from the exploration phase
    sol_listener.report_phase(OptimizationPhase::Compression);
    terminator.new_timeout(cmpr_config.time_limit);
    let mut cmpr_separator = Separator::new_constrained(
        expl_separator.instance,
        expl_separator.prob,
        next_rng(),
        cmpr_config.separator_config,
        constraints,
    )?;
    let cmpr_sol = compression_phase(
        &instance,
        &mut cmpr_separator,
        &final_explore_sol,
        sol_listener,
        terminator,
        cmpr_config,
    );

    sol_listener.report(ReportType::Final, &cmpr_sol, &instance);

    // Return the final compressed solution
    Ok(Some(cmpr_sol))
}
