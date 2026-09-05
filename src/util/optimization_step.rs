//! Opt-in semantic optimization snapshots, independent of progress/UI throttling.
//! One step is a completed search sweep or a solver state/decision transition,
//! not a collision test or sample evaluation.

use super::listener::SolutionListener;
use jagua_rs::geometry::DTransformation;
use jagua_rs::probs::spp::entities::SPProblem;

#[derive(Clone, Debug)]
pub struct ItemMove {
    pub item_id: usize,
    pub before: DTransformation,
    pub after: Option<DTransformation>,
    pub loss_before: f32,
    pub loss_after: f32,
    pub weighted_loss_before: f32,
    pub weighted_loss_after: f32,
    pub evaluations: usize,
    pub elapsed_ms: f64,
}

#[derive(Clone, Copy, Debug)]
pub struct StepMetric {
    pub name: &'static str,
    pub value: f64,
    pub unit: &'static str,
}

impl StepMetric {
    pub fn new(name: &'static str, value: impl Into<f64>, unit: &'static str) -> Self {
        Self {
            name,
            value: value.into(),
            unit,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct OptimizationStep<'a> {
    pub operation: &'static str,
    pub outcome: &'static str,
    pub reason: &'static str,
    pub metrics: &'a [StepMetric],
}

pub fn report_step(
    listener: &mut impl SolutionListener,
    operation: &'static str,
    outcome: &'static str,
    reason: &'static str,
    metrics: &[StepMetric],
    prob: &SPProblem,
) {
    if listener.wants_optimization_steps() {
        listener.report_optimization_step(
            OptimizationStep {
                operation,
                outcome,
                reason,
                metrics,
            },
            &prob.save(),
            &prob.instance,
        );
    }
}
