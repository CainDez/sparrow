use crate::eval::lbf_evaluator::LBFEvaluator;
use crate::eval::sample_eval::SampleEval;
use crate::sample::search::{search_placement, SampleConfig};
use crate::util::assertions;
use crate::util::terminator::Terminator;
use crate::util::listener::{DummySolListener, SolutionListener};
use crate::util::optimization_step::{report_step, StepMetric as M};
use itertools::Itertools;
use jagua_rs::entities::Instance;
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem};
use jagua_rs::Instant;
use log::debug;
use ordered_float::OrderedFloat;
use std::cmp::Reverse;
use std::iter;
use rand::rngs::Xoshiro256PlusPlus;

pub struct LBFBuilder {
    pub constraints: crate::constraints::PlacementConstraints,
    pub instance: SPInstance,
    pub prob: SPProblem,
    pub rng: Xoshiro256PlusPlus,
    pub sample_config: SampleConfig,
}

impl LBFBuilder {
    pub fn new(
        instance: SPInstance,
        rng: Xoshiro256PlusPlus,
        sample_config: SampleConfig,
    ) -> Self {
        let prob = SPProblem::new(instance.clone());

        Self {
            constraints: crate::constraints::PlacementConstraints::default(),
            instance,
            prob,
            rng,
            sample_config,
        }
    }

    pub fn construct(self, term: &impl Terminator) -> Self {
        self.construct_with_listener(term, &mut DummySolListener)
    }

    pub fn with_constraints(
        mut self,
        constraints: crate::constraints::PlacementConstraints,
    ) -> Result<Self, crate::constraints::ConstraintError> {
        constraints.initialize(&mut self.prob)?;
        self.constraints = constraints;
        Ok(self)
    }

    pub fn construct_with_listener(mut self, term: &impl Terminator, listener: &mut impl SolutionListener) -> Self {
        let start = Instant::now();
        let n_items = self.instance.items.len();
        let sorted_item_indices = (0..n_items)
            .sorted_by_cached_key(|id| {
                let item_shape = self.instance.item(*id).shape_cd.as_ref();
                let convex_hull_area = item_shape.surrogate().convex_hull_area;
                let diameter = item_shape.diameter;
                Reverse(OrderedFloat(convex_hull_area * diameter))
            })
            .flat_map(|id| {
                let missing_qty = self.prob.item_demand_qtys[id];
                iter::repeat_n(id, missing_qty)
            })
            .collect_vec();

        debug!("[CONSTR] placing items in order: {:?}",sorted_item_indices);

        for item_id in sorted_item_indices {
            // [PandaNest patch] construction is interruptible per item; a killed
            // run leaves a partial layout that callers discard via their own
            // cancellation checks before using the result.
            if term.kill() {
                break;
            }
            if self.constraints.is_empty() {
                self.place_item(item_id, listener);
            } else {
                self.place_constrained_item(item_id, term, listener);
            }
        }

        // [PandaNest patch] fit_strip would panic on an empty layout; a killed
        // construction that placed nothing keeps the problem's initial strip.
        if !self.prob.layout.placed_items.is_empty() {
            self.constraints.fit_initial_strip(&mut self.prob);
        }
        report_step(listener, "initial_layout", "constructed", "fit_initial_strip", &[], &self.prob);
        debug!("[CONSTR] placed all items in width: {:.3} (in {:?})",self.prob.strip_width(), start.elapsed());
        self
    }

    fn place_item(&mut self, item_id: usize, listener: &mut impl SolutionListener) {
        match self.find_placement(item_id) {
            Some(p_opt) => {
                self.prob.place_item(p_opt);
                report_step(listener, "initial_place", "accepted", "clear_candidate", &[M::new("item", item_id as f64, "index")], &self.prob);
                debug!("[CONSTR] placing item {}/{} with id {} at [{}]",self.prob.layout.placed_items.len(),self.instance.total_item_qty(),p_opt.item_id,p_opt.d_transf);
            }
            None => {
                debug!("[CONSTR] failed to place item with id {}, expanding strip width",item_id);
                self.prob.change_strip_width(self.prob.strip_width() * 1.2);
                report_step(listener, "initial_expand", "retry", "no_clear_candidate", &[M::new("item", item_id as f64, "index")], &self.prob);
                assert!(assertions::strip_width_is_in_check(&self.prob), "strip-width is running away (>{:.3}), item {item_id} does not seem to fit into the strip", self.prob.strip_width());          
                self.place_item(item_id, listener);
            }
        }
    }

    fn find_placement(&mut self, item_id: usize) -> Option<SPPlacement> {
        self.find_placement_until(item_id, None)
    }

    fn place_constrained_item(
        &mut self,
        item_id: usize,
        term: &impl Terminator,
        listener: &mut impl SolutionListener,
    ) {
        // 固定件可能远离原点，原先的总直径上限不适用；在固定跨度之外保留有界构造空间。
        let limit = self.constraints.minimum_width(&self.prob)
            + 2. * self
                .instance
                .items
                .iter()
                .map(|(i, q)| i.shape_cd.diameter * *q as f32)
                .sum::<f32>();
        while !term.kill() {
            if let Some(placement) = self.find_placement_until(item_id, term.timeout_at()) {
                if term.kill() {
                    return;
                }
                self.prob.place_item(placement);
                report_step(
                    listener,
                    "initial_place",
                    "accepted",
                    "clear_candidate",
                    &[M::new("item", item_id as f64, "index")],
                    &self.prob,
                );
                return;
            }
            let width = self.prob.strip_width() * 1.2;
            if width > limit || !width.is_finite() {
                report_step(
                    listener,
                    "initial_place",
                    "stopped",
                    "bounded_fixed_construction",
                    &[],
                    &self.prob,
                );
                return;
            }
            self.prob.change_strip_width(width);
            report_step(
                listener,
                "initial_expand",
                "retry",
                "no_clear_candidate",
                &[],
                &self.prob,
            );
        }
    }

    fn find_placement_until(
        &mut self,
        item_id: usize,
        timeout: Option<Instant>,
    ) -> Option<SPPlacement> {
        let layout = &self.prob.layout;
        let item = self.instance.item(item_id);
        let evaluator = LBFEvaluator::new(layout, item);

        let (best_sample, _) = search_placement(
            layout,
            item,
            None,
            evaluator,
            self.sample_config,
            &mut self.rng,
            timeout,
        );

        match best_sample {
            Some((d_transf, SampleEval::Clear { .. })) => {
                Some(SPPlacement { item_id, d_transf })
            }
            _ => None
        }
    }
}
