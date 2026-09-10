use jagua_rs::entities::Item;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::primitives::{Rect, SPolygon};
use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
use jagua_rs::geometry::{DTransformation, OriginalShape};
use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, Strip};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use sparrow::config::DEFAULT_SPARROW_CONFIG;
use sparrow::constraints::{FixedPlacement, PlacementConstraints};
use sparrow::optimizer::{optimize_constrained, separator::Separator};
use sparrow::quantify::tracker::CollisionTracker;
use sparrow::util::listener::{DummySolListener, ReportType, SolutionListener};
use sparrow::util::terminator::{BasicTerminator, Terminator};
use std::time::Duration;

fn item(id: usize, w: f32, h: f32) -> Item {
    Item::new(
        id,
        OriginalShape {
            shape: SPolygon::from(Rect::try_new(0., 0., w, h).unwrap()),
            pre_transform: DTransformation::empty(),
            modify_mode: ShapeModifyMode::Inflate,
            modify_config: ShapeModifyConfig::default(),
        },
        RotationRange::Discrete(vec![0.]),
        None,
        DEFAULT_SPARROW_CONFIG.cde_config.item_surrogate_config,
    )
    .unwrap()
}
fn problem(sizes: &[(f32, f32)]) -> SPProblem {
    let strip = Strip::new(
        100.,
        DEFAULT_SPARROW_CONFIG.cde_config,
        ShapeModifyConfig::default(),
        200.,
    )
    .unwrap();
    SPProblem::new(SPInstance::new(
        sizes
            .iter()
            .enumerate()
            .map(|(id, &(w, h))| (item(id, w, h), 1))
            .collect(),
        strip,
    ))
}
fn put(p: &mut SPProblem, id: usize, x: f32, y: f32) {
    p.place_item(SPPlacement {
        item_id: id,
        d_transf: DTransformation::new(0., (x, y)),
    });
}
fn pose(p: &SPProblem, id: usize) -> DTransformation {
    p.layout
        .placed_items
        .values()
        .find(|pi| pi.item_id == id)
        .unwrap()
        .d_transf
}
fn fixed(p: &SPProblem, id: usize) -> FixedPlacement {
    FixedPlacement::production(id, pose(p, id))
}
fn sep(p: SPProblem, c: PlacementConstraints) -> Separator {
    let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config;
    cfg.n_workers = 2;
    Separator::new_constrained(
        p.instance.clone(),
        p,
        Xoshiro256PlusPlus::seed_from_u64(42),
        cfg,
        c,
    )
    .unwrap()
}

fn hard_barrier_problem() -> (SPProblem, PlacementConstraints) {
    let mut p = problem(&[(2., 100.), (10., 10.), (10., 10.)]);
    put(&mut p, 0, 100., 0.);
    put(&mut p, 1, 105., 20.);
    put(&mut p, 2, 105., 20.);
    let c = PlacementConstraints::new(&p.instance, vec![FixedPlacement::boundary(0, pose(&p, 0))])
        .unwrap()
        .with_hard_x_barriers(&p.instance)
        .unwrap();
    (p, c)
}

#[test]
fn hard_barrier_rejects_crossing_pose_but_allows_atomic_cross_board_move() {
    let (p, c) = hard_barrier_problem();
    let mut s = sep(p, c.clone());
    let key = s
        .prob
        .layout
        .placed_items
        .iter()
        .find(|(_, pi)| pi.item_id == 1)
        .unwrap()
        .0;
    let before = pose(&s.prob, 1);
    assert_eq!(s.move_item(key, DTransformation::new(0., (96., 20.))), key);
    assert_eq!(pose(&s.prob, 1), before);
    let key = s.move_item(key, DTransformation::new(0., (70., 20.)));
    assert_eq!(pose(&s.prob, 1).translation().0, 70.);
    assert!(c.matches_solution(&s.prob.save()));
    // 外框仍为软约束：不会误把挡板规则扩大成完整无碰撞规则。
    s.move_item(key, DTransformation::new(0., (-20., 20.)));
    assert_eq!(pose(&s.prob, 1).translation().0, -20.);
}

#[test]
fn hard_barrier_guards_shrink_worker_evaluation_and_corrupt_rollback() {
    use sparrow::eval::sample_eval::{SampleEval, SampleEvaluator};
    use sparrow::eval::sep_evaluator::SeparationEvaluator;
    let (p, c) = hard_barrier_problem();
    let mut s = sep(p, c.clone());
    // 平移幅度会使位于 105 的副本落在挡板上；宽度仍可缩，非法移动须拒绝。
    assert!(s.change_strip_width(190., Some(100.)));
    assert_eq!(pose(&s.prob, 1).translation().0, 105.);
    assert!(c.matches_solution(&s.prob.save()));
    let key = s
        .prob
        .layout
        .placed_items
        .iter()
        .find(|(_, pi)| pi.item_id == 1)
        .unwrap()
        .0;
    let mut evaluator =
        SeparationEvaluator::new(&s.prob.layout, &s.instance.items[1].0, key, &s.ct);
    assert_eq!(
        evaluator.evaluate_sample(DTransformation::new(0., (96., 20.)), None),
        SampleEval::Invalid
    );
    let mut bad = s.prob.clone();
    bad.remove_item(key);
    put(&mut bad, 1, 96., 20.);
    assert!(!s.rollback(&bad.save(), None));
    let mut t = BasicTerminator::new();
    t.new_timeout(Duration::from_millis(300));
    s.separate(&t, &mut DummySolListener);
    assert!(c.matches_solution(&s.prob.save()));
    assert!(s.workers.iter().all(|w| c.matches_solution(&w.prob.save())));
}

#[test]
fn hard_barrier_sampler_covers_both_boards_and_allowed_rotations() {
    use sparrow::sample::uniform_sampler::UniformBBoxSampler;
    let (p, c) = hard_barrier_problem();
    let mut piece = item(1, 90., 40.);
    piece.allowed_rotation = RotationRange::Discrete(vec![0., std::f32::consts::FRAC_PI_2]);
    let bbox = p.layout.container.outer_cd.bbox;
    let regions = c.sample_regions(bbox).unwrap();
    assert_eq!(regions.len(), 2);
    let sampler = UniformBBoxSampler::new_in_regions(bbox, &piece, &regions).unwrap();
    let mut rng = Xoshiro256PlusPlus::seed_from_u64(42);
    let mut seen = [[false; 2]; 2];
    for _ in 0..500 {
        let pose = sampler.sample(&mut rng);
        assert!(c.allows_pose(&piece, pose));
        seen[usize::from(pose.translation().0 > 100.)][usize::from(pose.rotation() != 0.)] = true;
    }
    assert!(seen.iter().flatten().all(|v| *v));
}

#[test]
fn hard_barrier_projection_handles_large_coordinates_and_preserves_rotation() {
    let mut p = problem(&[(2., 100.), (20., 30.)]);
    p.change_strip_width(20000.);
    put(&mut p, 0, 13000., 0.);
    let c = PlacementConstraints::new(&p.instance, vec![FixedPlacement::boundary(0, pose(&p, 0))])
        .unwrap()
        .with_hard_x_barriers(&p.instance)
        .unwrap();
    let before = DTransformation::new(0., (13001.998, 17.));
    assert!(!c.allows_pose(&p.instance.items[1].0, before));
    let after = c.project_pose(&p.instance.items[1].0, before).unwrap();
    assert!(c.allows_pose(&p.instance.items[1].0, after));
    assert_eq!(
        (after.rotation(), after.translation().1),
        (before.rotation(), before.translation().1)
    );
    assert!((after.translation().0 - before.translation().0).abs() < 0.01);
    assert_eq!(
        c.project_pose(&p.instance.items[1].0, after).unwrap(),
        after
    );
}

#[test]
fn hard_barrier_every_reported_search_state_respects_boundaries() {
    use jagua_rs::probs::spp::entities::SPSolution;
    use sparrow::util::optimization_step::OptimizationStep;
    struct Audit {
        constraints: PlacementConstraints,
        states: usize,
    }
    impl SolutionListener for Audit {
        fn wants_optimization_steps(&self) -> bool {
            true
        }
        fn report(&mut self, _: ReportType, solution: &SPSolution, _: &SPInstance) {
            assert!(self.constraints.matches_solution(solution));
        }
        fn report_optimization_step(
            &mut self,
            _: OptimizationStep<'_>,
            solution: &SPSolution,
            _: &SPInstance,
        ) {
            assert!(self.constraints.matches_solution(solution));
            self.states += 1;
        }
        fn report_item_move(
            &mut self,
            movement: &sparrow::util::optimization_step::ItemMove,
            instance: &SPInstance,
        ) {
            if let Some(pose) = movement.after {
                assert!(
                    self.constraints
                        .allows_pose(&instance.items[movement.item_id].0, pose)
                );
            }
        }
    }
    let (p, c) = hard_barrier_problem();
    let mut cfg = DEFAULT_SPARROW_CONFIG;
    cfg.expl_cfg.time_limit = Duration::from_millis(200);
    cfg.cmpr_cfg.time_limit = Duration::from_millis(100);
    cfg.expl_cfg.separator_config.iter_no_imprv_limit = 5;
    cfg.cmpr_cfg.separator_config.iter_no_imprv_limit = 5;
    let mut audit = Audit {
        constraints: c.clone(),
        states: 0,
    };
    let result = optimize_constrained(
        p.instance.clone(),
        Xoshiro256PlusPlus::seed_from_u64(42),
        &mut audit,
        &mut BasicTerminator::new(),
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        Some(&p.save()),
        c,
    )
    .unwrap();
    assert!(result.is_some());
    assert!(audit.states > 10);
}

#[test]
fn fixed_pose_survives_shrink_move_and_rollback() {
    let mut p = problem(&[(10., 10.), (10., 10.)]);
    put(&mut p, 0, 140., 20.);
    put(&mut p, 1, 175., 20.);
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
    let mut s = sep(p, c);
    let original = pose(&s.prob, 0);
    assert!(s.change_strip_width(170., Some(100.)));
    assert_eq!(pose(&s.prob, 0), original);
    assert_eq!(pose(&s.prob, 1).translation(), (145., 20.));
    let good = s.prob.save();
    let pk = s
        .prob
        .layout
        .placed_items
        .iter()
        .find(|(_, p)| p.item_id == 0)
        .unwrap()
        .0;
    s.move_item(pk, DTransformation::new(0., (10., 10.)));
    assert_eq!(pose(&s.prob, 0), original);
    assert!(s.rollback(&good, None));
    assert_eq!(pose(&s.prob, 0), original);
    assert!(!s.change_strip_width(130., None));
    assert_eq!(s.prob.strip_width(), 170.);
}

#[test]
fn workers_resolve_movable_collision_without_moving_fixed_item() {
    let mut p = problem(&[(20., 20.), (20., 20.)]);
    put(&mut p, 0, 30., 30.);
    put(&mut p, 1, 30., 30.);
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
    let mut s = sep(p, c);
    let original = pose(&s.prob, 0);
    let mut t = BasicTerminator::new();
    t.new_timeout(Duration::from_secs(2));
    s.separate(&t, &mut DummySolListener);
    assert_eq!(pose(&s.prob, 0), original);
    assert_ne!(pose(&s.prob, 1), original);
    assert_eq!(s.ct.get_total_loss(), 0.);
    for worker in &s.workers {
        assert_eq!(pose(&worker.prob, 0), original);
    }
}

#[test]
fn rejects_invalid_and_ambiguous_fixed_definitions() {
    let p = problem(&[(10., 10.)]);
    let f = FixedPlacement::production(0, DTransformation::new(0., (20., 20.)));
    assert!(PlacementConstraints::new(&p.instance, vec![f.clone(), f]).is_err());
    assert!(
        PlacementConstraints::new(
            &p.instance,
            vec![FixedPlacement::production(1, DTransformation::empty())]
        )
        .is_err()
    );
    assert!(
        PlacementConstraints::new(
            &p.instance,
            vec![FixedPlacement::production(
                0,
                DTransformation::new(0., (f32::INFINITY, 20.))
            )]
        )
        .is_err()
    );
    assert!(
        PlacementConstraints::new(
            &p.instance,
            vec![FixedPlacement::production(
                0,
                DTransformation::new(0.4, (20., 20.))
            )]
        )
        .is_err()
    );
    let mut i = p.instance.clone();
    i.items[0].1 = 2;
    assert!(
        PlacementConstraints::new(
            &i,
            vec![FixedPlacement::production(0, DTransformation::empty())]
        )
        .is_err()
    );
}

#[test]
fn rejects_fixed_conflicts_and_corrupt_restore() {
    let mut p = problem(&[(20., 20.), (20., 20.)]);
    put(&mut p, 0, 30., 30.);
    put(&mut p, 1, 30., 30.);
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0), fixed(&p, 1)]).unwrap();
    let cfg = DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config;
    assert!(
        Separator::new_constrained(
            p.instance.clone(),
            p.clone(),
            Xoshiro256PlusPlus::seed_from_u64(42),
            cfg,
            c
        )
        .is_err()
    );
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
    let mut s = sep(p, c);
    let mut bad = s.prob.save();
    bad.layout_snapshot
        .placed_items
        .values_mut()
        .find(|pi| pi.item_id == 0)
        .unwrap()
        .d_transf = DTransformation::empty();
    assert!(!s.rollback(&bad, None));
    assert_eq!(pose(&s.prob, 0).translation(), (30., 30.));
}

#[test]
fn boundary_can_touch_container_but_still_blocks_real_parts() {
    let mut p = problem(&[(2., 100.), (20., 20.)]);
    put(&mut p, 0, 100., 0.);
    put(&mut p, 1, 95., 30.);
    assert!(
        CollisionTracker::new(&p.layout)
            .get_container_loss(p.layout.placed_items.keys().next().unwrap())
            > 0.
    );
    let c = PlacementConstraints::new(&p.instance, vec![FixedPlacement::boundary(0, pose(&p, 0))])
        .unwrap();
    let mut s = sep(p, c);
    let wall = pose(&s.prob, 0);
    assert!(s.ct.get_total_loss() > 0.);
    let mut t = BasicTerminator::new();
    t.new_timeout(Duration::from_secs(2));
    s.separate(&t, &mut DummySolListener);
    assert_eq!(pose(&s.prob, 0), wall);
    assert_eq!(s.ct.get_total_loss(), 0.);
    let real = s
        .prob
        .layout
        .placed_items
        .values()
        .find(|pi| pi.item_id == 1)
        .unwrap()
        .shape
        .bbox;
    assert!(real.x_max < 100. || real.x_min > 102.);
}

#[derive(Default)]
struct CheckingListener {
    fixed_id: usize,
    expected: Option<DTransformation>,
    steps: usize,
    reports: usize,
    operations: Vec<String>,
}
impl CheckingListener {
    fn check(&mut self, s: &jagua_rs::probs::spp::entities::SPSolution) {
        let pi = s
            .layout_snapshot
            .placed_items
            .values()
            .find(|pi| pi.item_id == self.fixed_id)
            .unwrap();
        assert_eq!(pi.d_transf, self.expected.unwrap());
        self.steps += 1;
    }
}
impl SolutionListener for CheckingListener {
    fn wants_optimization_steps(&self) -> bool {
        true
    }
    fn report(
        &mut self,
        _: ReportType,
        s: &jagua_rs::probs::spp::entities::SPSolution,
        _: &SPInstance,
    ) {
        self.check(s);
        self.reports += 1;
    }
    fn report_optimization_step(
        &mut self,
        step: sparrow::util::optimization_step::OptimizationStep<'_>,
        s: &jagua_rs::probs::spp::entities::SPSolution,
        _: &SPInstance,
    ) {
        self.check(s);
        self.operations.push(step.operation.to_owned());
    }
}

#[test]
fn cold_optimize_initializes_fixed_item_and_preserves_it_in_every_phase() {
    let p = problem(&[(20., 20.), (15., 15.), (25., 10.)]);
    let expected = DTransformation::new(0., (60., 30.));
    let c = PlacementConstraints::new(&p.instance, vec![FixedPlacement::production(0, expected)])
        .unwrap();
    let mut cfg = DEFAULT_SPARROW_CONFIG;
    cfg.expl_cfg.time_limit = Duration::from_millis(100);
    cfg.cmpr_cfg.time_limit = Duration::from_millis(100);
    cfg.expl_cfg.separator_config.n_workers = 2;
    cfg.cmpr_cfg.separator_config.n_workers = 2;
    let mut listener = CheckingListener {
        expected: Some(expected),
        ..Default::default()
    };
    let result = optimize_constrained(
        p.instance,
        Xoshiro256PlusPlus::seed_from_u64(42),
        &mut listener,
        &mut BasicTerminator::new(),
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        None,
        c,
    )
    .unwrap()
    .unwrap();
    listener.check(&result);
    assert!(listener.steps > 5);
    assert!(listener.reports > 0);
}

#[test]
fn fixed_pose_survives_failed_pool_disruption_and_compression() {
    let mut p = problem(&[(20., 20.), (90., 90.), (90., 90.)]);
    p.change_strip_width(260.);
    put(&mut p, 0, 10., 70.);
    put(&mut p, 1, 40., 5.);
    put(&mut p, 2, 150., 5.);
    let expected = pose(&p, 0);
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
    let mut cfg = DEFAULT_SPARROW_CONFIG;
    cfg.expl_cfg.target_width = Some(190.);
    cfg.expl_cfg.time_limit = Duration::from_secs(2);
    cfg.expl_cfg.max_conseq_failed_attempts = Some(2);
    cfg.expl_cfg.separator_config.n_workers = 2;
    cfg.expl_cfg.separator_config.strike_limit = 1;
    cfg.expl_cfg.separator_config.iter_no_imprv_limit = 1;
    cfg.expl_cfg
        .separator_config
        .sample_config
        .n_container_samples = 4;
    cfg.expl_cfg
        .separator_config
        .sample_config
        .n_focussed_samples = 4;
    cfg.expl_cfg.separator_config.sample_config.n_coord_descents = 1;
    cfg.cmpr_cfg.time_limit = Duration::from_millis(20);
    cfg.cmpr_cfg.separator_config = cfg.expl_cfg.separator_config;
    let mut listener = CheckingListener {
        expected: Some(expected),
        ..Default::default()
    };
    let initial = p.save();
    let result = optimize_constrained(
        p.instance,
        Xoshiro256PlusPlus::seed_from_u64(42),
        &mut listener,
        &mut BasicTerminator::new(),
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        Some(&initial),
        c,
    )
    .unwrap()
    .unwrap();
    listener.check(&result);
    assert!(listener.operations.iter().any(|op| op == "disrupt"));
    assert!(listener.operations.iter().any(|op| op == "pool_rollback"));
    assert!(
        listener
            .operations
            .iter()
            .any(|op| op == "compression_shrink")
    );
}

#[test]
fn fixed_real_outside_container_is_rejected() {
    let mut p = problem(&[(10., 10.)]);
    put(&mut p, 0, -1., 10.);
    let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
    assert!(
        Separator::new_constrained(
            p.instance.clone(),
            p,
            Xoshiro256PlusPlus::seed_from_u64(42),
            DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config,
            c
        )
        .is_err()
    );
}

struct Killed;
impl Terminator for Killed {
    fn kill(&self) -> bool {
        true
    }
    fn new_timeout(&mut self, _: Duration) {}
    fn timeout_at(&self) -> Option<jagua_rs::Instant> {
        None
    }
}

#[test]
fn cancelled_complete_fixed_start_is_retained_but_overlap_is_not_certified() {
    for overlap in [false, true] {
        let mut p = problem(&[(20., 20.), (20., 20.)]);
        put(&mut p, 0, 30., 30.);
        put(&mut p, 1, if overlap { 30. } else { 80. }, 30.);
        let c = PlacementConstraints::new(&p.instance, vec![fixed(&p, 0)]).unwrap();
        let initial = p.save();
        let cfg = DEFAULT_SPARROW_CONFIG;
        let result = optimize_constrained(
            p.instance,
            Xoshiro256PlusPlus::seed_from_u64(42),
            &mut DummySolListener,
            &mut Killed,
            &cfg.expl_cfg,
            &cfg.cmpr_cfg,
            Some(&initial),
            c,
        )
        .unwrap();
        assert_eq!(result.is_some(), !overlap);
        if let Some(s) = result {
            assert_eq!(
                s.layout_snapshot
                    .placed_items
                    .values()
                    .find(|pi| pi.item_id == 0)
                    .unwrap()
                    .d_transf,
                DTransformation::new(0., (30., 30.))
            );
        }
    }
}

#[test]
fn all_fixed_nonzero_angle_stops_at_fixed_extent() {
    let mut part = item(0, 10., 20.);
    let angle = std::f32::consts::FRAC_PI_2;
    part.allowed_rotation = RotationRange::Discrete(vec![angle]);
    let strip = Strip::new(
        100.,
        DEFAULT_SPARROW_CONFIG.cde_config,
        ShapeModifyConfig::default(),
        200.,
    )
    .unwrap();
    let instance = SPInstance::new(vec![(part, 1)], strip);
    let expected = DTransformation::new(angle, (60., 30.));
    let constraints =
        PlacementConstraints::new(&instance, vec![FixedPlacement::production(0, expected)])
            .unwrap();
    let mut listener = CheckingListener {
        expected: Some(expected),
        ..Default::default()
    };
    let started = std::time::Instant::now();
    let mut cfg = DEFAULT_SPARROW_CONFIG;
    cfg.expl_cfg.time_limit = Duration::from_secs(3);
    cfg.cmpr_cfg.time_limit = Duration::from_secs(3);
    let result = optimize_constrained(
        instance,
        Xoshiro256PlusPlus::seed_from_u64(42),
        &mut listener,
        &mut BasicTerminator::new(),
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        None,
        constraints,
    )
    .unwrap()
    .unwrap();
    listener.check(&result);
    assert!(
        started.elapsed() < Duration::from_secs(1),
        "全部固定时不能耗尽搜索预算"
    );
}

#[test]
fn far_away_fixed_pose_and_unplaceable_movable_construct_with_bounds() {
    let p = problem(&[(10., 10.), (20., 150.)]);
    let expected = DTransformation::new(0., (10000., 20.));
    let constraints =
        PlacementConstraints::new(&p.instance, vec![FixedPlacement::production(0, expected)])
            .unwrap();
    let mut cfg = DEFAULT_SPARROW_CONFIG;
    cfg.expl_cfg.time_limit = Duration::from_millis(100);
    let mut listener = CheckingListener {
        expected: Some(expected),
        ..Default::default()
    };
    let result = optimize_constrained(
        p.instance,
        Xoshiro256PlusPlus::seed_from_u64(42),
        &mut listener,
        &mut BasicTerminator::new(),
        &cfg.expl_cfg,
        &cfg.cmpr_cfg,
        None,
        constraints,
    )
    .unwrap();
    assert!(result.is_none(), "无法放置完整需求不能认证为成功");
}

#[test]
fn mismatched_instance_and_tracker_constraints_are_rejected() {
    let mut p = problem(&[(2., 100.), (10., 10.)]);
    put(&mut p, 0, 100., 0.);
    put(&mut p, 1, 30., 30.);
    let c = PlacementConstraints::new(&p.instance, vec![FixedPlacement::boundary(0, pose(&p, 0))])
        .unwrap();
    let plain_tracker = CollisionTracker::new(&p.layout);
    let mut s = sep(p, c.clone());
    let saved = s.prob.save();
    assert!(!s.rollback(&saved, Some(&plain_tracker)));
    let mut other = problem(&[]);
    assert!(c.initialize(&mut other).is_err());
}

#[test]
fn crossing_fixed_width_floor_attempts_the_floor_and_evacuates_a_narrow_tail() {
    for compression in [false, true] {
        let mut p = problem(&[(2., 100.), (1., 10.)]);
        p.change_strip_width(104.);
        put(&mut p, 0, 100., 0.);
        put(&mut p, 1, 102.5, 20.);
        let fixed_pose = pose(&p, 0);
        let constraints =
            PlacementConstraints::new(&p.instance, vec![FixedPlacement::boundary(0, fixed_pose)])
                .unwrap();
        let initial = p.save();
        let mut separator = sep(p, constraints.clone());
        let mut term = BasicTerminator::new();
        term.new_timeout(Duration::from_secs(2));
        let mut listener = DummySolListener;
        let instance = separator.instance.clone();
        let solution = if compression {
            let mut cfg = DEFAULT_SPARROW_CONFIG.cmpr_cfg;
            cfg.shrink_range = (0.1, 0.1);
            sparrow::optimizer::compress::compression_phase(
                &instance,
                &mut separator,
                &initial,
                &mut listener,
                &term,
                &cfg,
            )
        } else {
            let mut cfg = DEFAULT_SPARROW_CONFIG.expl_cfg;
            cfg.shrink_step = 0.1;
            sparrow::optimizer::explore::exploration_phase(
                &instance,
                &mut separator,
                &mut listener,
                &term,
                &cfg,
                false,
            )
            .pop()
            .unwrap()
        };
        assert_eq!(
            solution.strip_width(),
            102.,
            "缩长越过固定跨度时必须先尝试精确下界"
        );
        assert!(constraints.matches_solution(&solution));
        let real = solution
            .layout_snapshot
            .placed_items
            .values()
            .find(|p| p.item_id == 1)
            .unwrap();
        assert!(real.shape.bbox.x_max <= 100., "尾件必须完整迁过挡板");
    }
}
