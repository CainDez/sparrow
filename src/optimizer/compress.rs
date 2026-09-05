use crate::config::{CompressionConfig, ShrinkDecayStrategy};
use crate::optimizer::separator::Separator;
use crate::util::listener::{ReportType, SolutionListener};
use crate::util::optimization_step::{report_step, StepMetric as M};
use crate::util::terminator::Terminator;
use jagua_rs::probs::spp::entities::{SPInstance, SPSolution};
use jagua_rs::Instant;
use log::info;
use rand::{Rng, RngExt};

/// Algorithm 13 from https://doi.org/10.48550/arXiv.2509.13329
pub fn compression_phase(
    instance: &SPInstance,
    sep: &mut Separator,
    init_sol: &SPSolution,
    sol_listener: &mut impl SolutionListener,
    term: &impl Terminator,
    config: &CompressionConfig
) -> SPSolution {
    let mut best_sol = init_sol.clone();
    let start = Instant::now();
    let mut n_failed_attempts = 0;

    // Create the function to calculate the shrink step size.
    let shrink_step_size = |n_failed_attempts: i32| -> f32 {
        match config.shrink_decay {
            ShrinkDecayStrategy::TimeBased => {
                let range = config.shrink_range.1 - config.shrink_range.0;
                let elapsed = start.elapsed();
                let ratio = elapsed.as_secs_f32() / config.time_limit.as_secs_f32();
                config.shrink_range.0 + ratio * range
            }
            ShrinkDecayStrategy::FailureBased(r) => {
                config.shrink_range.0 * r.powi(n_failed_attempts)
            }
        }
    };

    // As long as the shrink step size is above the minimum, keep attempting to compress
    loop {
        if term.kill() {
            report_step(sol_listener, "compression_stop", "stopped", "termination_observed", &[], &sep.prob);
            break;
        }
        let step = shrink_step_size(n_failed_attempts);
        if !(step >= config.shrink_range.1) {
            report_step(sol_listener, "compression_stop", "stopped", "minimum_shrink_step", &[M::new("shrink_ratio", step, "ratio")], &sep.prob);
            break;
        }
        sol_listener.report_compression_progress(step);
        match attempt_to_compress(sep, &best_sol, step, term, sol_listener) {
            Some(compacted_sol) => {
                info!("[CMPR] success at {:.3}% ({:.3} | {:.3}%)", step * 100.0, compacted_sol.strip_width(), compacted_sol.density(instance) * 100.0);
                sol_listener.report(ReportType::CmprFeas, &compacted_sol, instance);
                best_sol = compacted_sol;
            }
            None => {
                info!("[CMPR] failed at {:.3}%", step * 100.0);
                n_failed_attempts += 1;
            }
        }
    }
    info!("[CMPR] finished, compressed from {:.3}% to {:.3}% (+{:.3}%)", init_sol.density(instance) * 100.0, best_sol.density(instance) * 100.0, (best_sol.density(instance) - init_sol.density(instance)) * 100.0);
    best_sol
}


fn attempt_to_compress(sep: &mut Separator, init_sol: &SPSolution, r_shrink: f32, term: &impl Terminator, sol_listener: &mut impl SolutionListener) -> Option<SPSolution> {
    // Restore to the initial solution and width
    sep.change_strip_width(init_sol.strip_width(), None);
    sep.rollback(init_sol, None);
    report_step(sol_listener, "compression_restore", "restored", "start_from_best_feasible", &[], &sep.prob);

    // Shrink the container by the provided amount at a random position
    let new_width = init_sol.strip_width() * (1.0 - r_shrink);
    let split_pos = sep.rng.random_range(0.0..sep.prob.strip_width());
    sep.change_strip_width(new_width, Some(split_pos));
    report_step(sol_listener, "compression_shrink", "attempt", "random_split", &[
        M::new("width_before", init_sol.strip_width(), "mm"), M::new("width", new_width, "mm"),
        M::new("split_position", split_pos, "mm"), M::new("shrink_ratio", r_shrink, "ratio"),
    ], &sep.prob);

    // Try to separate layout, if all collisions are eliminated, return the solution
    let (compacted_sol, ot) = sep.separate(term, sol_listener);
    if sol_listener.wants_optimization_steps() {
        sol_listener.report_optimization_step(crate::util::optimization_step::OptimizationStep {
            operation: "compression_result",
            outcome: if ot.get_total_loss() == 0.0 { "accepted" } else { "rejected" },
            reason: if ot.get_total_loss() == 0.0 { "zero_collision_loss" } else { "remaining_collisions_keep_incumbent" },
            metrics: &[M::new("loss", ot.get_total_loss(), "loss"), M::new("width_before", init_sol.strip_width(), "mm")],
        }, &compacted_sol, &sep.instance);
    }
    match ot.get_total_loss() == 0.0 {
        true => Some(compacted_sol),
        false => None,
    }
}
