use jagua_rs::entities::Item;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::primitives::{Rect, SPolygon};
use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
use jagua_rs::geometry::{DTransformation, OriginalShape};
use jagua_rs::probs::spp::entities::{SPInstance, Strip};
use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};
use sparrow::{config::DEFAULT_SPARROW_CONFIG, sample::uniform_sampler::UniformBBoxSampler};

fn item(w: f32, h: f32) -> Item {
    Item::new(
        0,
        OriginalShape {
            shape: SPolygon::from(Rect::try_new(0., 0., w, h).unwrap()),
            pre_transform: DTransformation::empty(),
            modify_mode: ShapeModifyMode::Inflate,
            modify_config: ShapeModifyConfig::default(),
        },
        RotationRange::None,
        None,
        DEFAULT_SPARROW_CONFIG.cde_config.item_surrogate_config,
    )
    .unwrap()
}

#[test]
fn exact_height_fit_retains_a_valid_sampling_axis() {
    let piece = item(10., 100.);
    let bbox = Rect::try_new(0., 0., 100., 100.).unwrap();
    let sampler =
        UniformBBoxSampler::new(bbox, &piece, bbox).expect("恰好匹配幅宽的零件必须可采样");
    let sample = sampler.sample(&mut Xoshiro256PlusPlus::seed_from_u64(42));
    assert_eq!(sample.translation().1, 0.);
    assert!(sample.translation().0 >= 0. && sample.translation().0 <= 90.);
}

#[test]
fn impossible_initial_construction_returns_controlled_no_candidate() {
    let strip = Strip::new(
        10.,
        DEFAULT_SPARROW_CONFIG.cde_config,
        ShapeModifyConfig::default(),
        100.,
    )
    .unwrap();
    let instance = SPInstance::new(vec![(item(10., 20.), 1)], strip);
    let result = std::panic::catch_unwind(|| {
        sparrow::optimizer::optimize(
            instance,
            Xoshiro256PlusPlus::seed_from_u64(42),
            &mut sparrow::util::listener::DummySolListener,
            &mut sparrow::util::terminator::BasicTerminator::new(),
            &DEFAULT_SPARROW_CONFIG.expl_cfg,
            &DEFAULT_SPARROW_CONFIG.cmpr_cfg,
            None,
        )
    });
    assert!(result.is_ok(), "启发式构造失败不能 panic");
    assert!(result.unwrap().is_none(), "失败不能形成可行证书");
}
