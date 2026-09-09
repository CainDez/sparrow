use jagua_rs::geometry::DTransformation;
use jagua_rs::io::{
    ext_repr::{ExtItem as BaseItem, ExtShape},
    import::Importer,
};
use jagua_rs::probs::spp::{
    entities::{SPInstance, SPPlacement, SPProblem},
    io::{
        ext_repr::{ExtItem, ExtSPInstance},
        import_instance,
    },
};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use sparrow::{
    config::DEFAULT_SPARROW_CONFIG,
    optimizer::{explore::exploration_phase, separator::Separator},
    quantify::tracker::CollisionTracker,
    util::{listener::DummySolListener, terminator::Terminator},
};
use std::time::Duration;

struct Bounded {
    killed: bool,
}
impl Terminator for Bounded {
    fn kill(&self) -> bool {
        self.killed
    }
    fn new_timeout(&mut self, _: Duration) {}
    fn timeout_at(&self) -> Option<jagua_rs::Instant> {
        None
    }
}

fn problem(overlap: bool) -> (SPInstance, SPProblem) {
    let importer = Importer::new(DEFAULT_SPARROW_CONFIG.cde_config, None, None, None);
    let instance = import_instance(
        &importer,
        &ExtSPInstance {
            name: "热启动可行证书回归".into(),
            strip_height: 100.0,
            items: (0..2)
                .map(|id| ExtItem {
                    base: BaseItem {
                        id,
                        allowed_orientations: Some(vec![0.0]),
                        min_quality: None,
                        shape: ExtShape::Rectangle {
                            x_min: 0.0,
                            y_min: 0.0,
                            width: 10.0,
                            height: 10.0,
                        },
                    },
                    demand: 1,
                })
                .collect(),
        },
    )
    .unwrap();
    let mut prob = SPProblem::new(instance.clone());
    prob.change_strip_width(100.0);
    for item_id in 0..2 {
        prob.place_item(SPPlacement {
            item_id,
            d_transf: DTransformation::new(
                0.0,
                (
                    20.0 + if overlap { 0.0 } else { item_id as f32 * 30.0 },
                    20.0,
                ),
            ),
        });
    }
    (instance, prob)
}

#[test]
fn overlapping_target_is_repaired_and_equal_width_solution_is_saved() {
    let (instance, prob) = problem(true);
    let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg;
    cfg.target_width = Some(100.0);
    cfg.max_conseq_failed_attempts = Some(1);
    cfg.separator_config.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(42),
        cfg.separator_config,
    );
    assert!(sep.ct.get_total_loss() > 0.0);
    let sols = exploration_phase(
        &instance,
        &mut sep,
        &mut DummySolListener,
        &Bounded { killed: false },
        &cfg,
        true,
    );
    assert_eq!(sols.len(), 1);
    let mut restored = SPProblem::new(instance);
    restored.restore(&sols[0]);
    assert_eq!(
        CollisionTracker::new(&restored.layout).get_total_loss(),
        0.0
    );
    assert_eq!(sols[0].strip_width(), 100.0);
}

#[test]
fn killed_overlapping_start_is_not_a_feasible_fallback() {
    let (instance, prob) = problem(true);
    let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg;
    cfg.target_width = Some(100.0);
    cfg.separator_config.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(42),
        cfg.separator_config,
    );
    assert!(
        exploration_phase(
            &instance,
            &mut sep,
            &mut DummySolListener,
            &Bounded { killed: true },
            &cfg,
            true
        )
        .is_empty()
    );
}

#[test]
fn valid_target_remains_an_immediate_feasible_fallback() {
    let (instance, prob) = problem(false);
    let initial = prob.save();
    let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg;
    cfg.target_width = Some(100.0);
    cfg.separator_config.n_workers = 1;
    let mut sep = Separator::new(
        instance.clone(),
        prob,
        Xoshiro256PlusPlus::seed_from_u64(42),
        cfg.separator_config,
    );
    let sols = exploration_phase(
        &instance,
        &mut sep,
        &mut DummySolListener,
        &Bounded { killed: true },
        &cfg,
        true,
    );
    assert_eq!(sols.len(), 1);
    assert_eq!(sols[0].strip_width(), initial.strip_width());
    assert_eq!(sep.ct.get_total_loss(), 0.0);
}

#[test]
fn real_separation_search_preserves_sampling_after_first_clear_sample() {
    use jagua_rs::entities::Instance;
    use sparrow::eval::{
        sample_eval::{SampleEval, SampleEvaluator},
        sep_evaluator::SeparationEvaluator,
    };
    use std::{cell::Cell, rc::Rc};
    struct Tracked<E> {
        inner: E,
        first: Rc<Cell<usize>>,
    }
    impl<E: SampleEvaluator> SampleEvaluator for Tracked<E> {
        fn evaluate_sample(
            &mut self,
            dt: DTransformation,
            upper: Option<SampleEval>,
        ) -> SampleEval {
            let eval = self.inner.evaluate_sample(dt, upper);
            if eval == (SampleEval::Clear { loss: 0.0 }) && self.first.get() == 0 {
                self.first.set(self.inner.n_evals());
            }
            eval
        }
        fn n_evals(&self) -> usize {
            self.inner.n_evals()
        }
    }
    {
        let (instance, prob) = problem(true);
        let ct = CollisionTracker::new(&prob.layout);
        let pk = prob.layout.placed_items.keys().next().unwrap();
        let item = instance.item(prob.layout.placed_items[pk].item_id);
        let first = Rc::new(Cell::new(0));
        let evaluator = Tracked {
            inner: SeparationEvaluator::new(&prob.layout, item, pk, &ct),
            first: first.clone(),
        };
        let config = DEFAULT_SPARROW_CONFIG
            .expl_cfg
            .separator_config
            .sample_config;
        let (best, evals) = sparrow::sample::search::search_placement(
            &prob.layout,
            item,
            Some(pk),
            evaluator,
            config,
            &mut Xoshiro256PlusPlus::seed_from_u64(42),
            None,
        );
        assert_eq!(best.unwrap().1, SampleEval::Clear { loss: 0.0 });
        assert!(first.get() > 0);
        // 单件首次 Clear 不代表整板最优；提前结束曾使 dagli 的 K′ 明显退化。
        assert_eq!(evals, 164);
        assert!(evals > first.get());
        println!(
            "首次零损失 = {}，保留完整采样评估 = {}",
            first.get(),
            evals
        );
    }
}

#[test]
fn exact_noop_keeps_original_remove_place_semantics() {
    let (instance, prob) = problem(true);
    let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = 1;
    let mut sep = Separator::new(instance, prob, Xoshiro256PlusPlus::seed_from_u64(42), cfg);
    let worker = &mut sep.workers[0];
    let pk = worker.prob.layout.placed_items.keys().next().unwrap();
    let pose = worker.prob.layout.placed_items[pk].d_transf;
    let next = worker.move_item(pk, pose);
    // 几何相同不代表搜索状态等价；跳过删除/插入曾导致 shapes1 K′ 明显退化。
    assert_ne!(next, pk, "保留原搜索的 SlotMap 更新语义");
    assert!(sparrow::util::assertions::tracker_matches_layout(
        &worker.ct,
        &worker.prob.layout
    ));
    let moved = worker.move_item(next, DTransformation::new(0.0, (70.0, 70.0)));
    assert_ne!(moved, next);
    assert_eq!(worker.ct.get_loss(moved), 0.0);
    assert!(sparrow::util::assertions::tracker_matches_layout(
        &worker.ct,
        &worker.prob.layout
    ));
}

#[test]
fn missing_identity_or_disallowed_rotation_is_not_a_feasible_certificate() {
    for rotate in [false, true] {
        let (instance, mut prob) = problem(false);
        let pk = prob.layout.placed_items.keys().next().unwrap();
        let old = prob.remove_item(pk);
        if rotate {
            prob.place_item(SPPlacement {
                item_id: old.item_id,
                d_transf: DTransformation::new(0.3, (20.0, 20.0)),
            });
        }
        let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg;
        cfg.target_width = Some(100.0);
        cfg.separator_config.n_workers = 1;
        let mut sep = Separator::new(
            instance.clone(),
            prob,
            Xoshiro256PlusPlus::seed_from_u64(42),
            cfg.separator_config,
        );
        assert!(
            exploration_phase(
                &instance,
                &mut sep,
                &mut DummySolListener,
                &Bounded { killed: true },
                &cfg,
                true
            )
            .is_empty()
        );
    }
}
