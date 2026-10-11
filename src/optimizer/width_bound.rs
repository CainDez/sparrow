//! 上游 #162 的必要宽度下界；内部挡板不作为生产零件参与面积或旋转界。
use super::separator::Separator;
use jagua_rs::geometry::Transformation;
use jagua_rs::geometry::geo_enums::RotationRange;
use jagua_rs::geometry::geo_traits::TransformableFrom;

pub(super) fn strip_width_lower_bound(sep: &Separator) -> f32 {
    const REL_TOL: f32 = 1e-6;
    let container = sep.prob.layout.container.outer_cd.bbox;
    let inset = sep.prob.strip_width() - container.width();
    let real = sep
        .instance
        .items
        .iter()
        .filter(|(item, _)| !sep.ct.constraints.is_boundary(item.id));
    let area: f64 = real
        .clone()
        .map(|(item, quantity)| f64::from(item.shape_cd.area) * *quantity as f64)
        .sum();
    let mut lower = (area / f64::from(container.height())) as f32 * (1. - REL_TOL);
    for (item, _) in real {
        let rotations = match &item.allowed_rotation {
            RotationRange::None => &[0.][..],
            RotationRange::Discrete(angles) => angles.as_slice(),
            RotationRange::Continuous => continue,
        };
        let tolerance = item.shape_cd.diameter * REL_TOL;
        let mut shape = item.shape_cd.as_ref().clone();
        let width = rotations
            .iter()
            .filter_map(|&angle| {
                let bbox = shape
                    .transform_from(
                        item.shape_cd.as_ref(),
                        &Transformation::from_rotation(angle),
                    )
                    .bbox;
                (bbox.height() <= container.height() + tolerance)
                    .then_some((bbox.width() - tolerance).max(0.))
            })
            .fold(f32::INFINITY, f32::min);
        lower = lower.max(width);
    }
    lower + inset
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::DEFAULT_SPARROW_CONFIG;
    use crate::constraints::{FixedPlacement, PlacementConstraints};
    use jagua_rs::entities::Item;
    use jagua_rs::geometry::primitives::{Rect, SPolygon};
    use jagua_rs::geometry::shape_modification::{ShapeModifyConfig, ShapeModifyMode};
    use jagua_rs::geometry::{DTransformation, OriginalShape};
    use jagua_rs::probs::spp::entities::{SPInstance, SPPlacement, SPProblem, Strip};
    use rand::{SeedableRng, rngs::Xoshiro256PlusPlus};

    fn item(id: usize, w: f32, h: f32, rotations: RotationRange) -> Item {
        Item::new(
            id,
            OriginalShape {
                shape: SPolygon::from(Rect::try_new(0., 0., w, h).unwrap()),
                pre_transform: DTransformation::empty(),
                modify_mode: ShapeModifyMode::Inflate,
                modify_config: ShapeModifyConfig::default(),
            },
            rotations,
            None,
            DEFAULT_SPARROW_CONFIG.cde_config.item_surrogate_config,
        )
        .unwrap()
    }

    fn separator(items: Vec<(Item, usize)>, boundary: bool) -> Separator {
        let strip = Strip::new(
            100.,
            DEFAULT_SPARROW_CONFIG.cde_config,
            ShapeModifyConfig::default(),
            200.,
        )
        .unwrap();
        let instance = SPInstance::new(items, strip);
        let mut prob = SPProblem::new(instance.clone());
        let constraints = if boundary {
            let pose = DTransformation::new(0., (150., 0.));
            prob.place_item(SPPlacement {
                item_id: 0,
                d_transf: pose,
            });
            PlacementConstraints::new(&instance, vec![FixedPlacement::boundary(0, pose)]).unwrap()
        } else {
            PlacementConstraints::default()
        };
        let mut config = DEFAULT_SPARROW_CONFIG.expl_cfg.separator_config;
        config.n_workers = 1;
        Separator::new_constrained(
            instance,
            prob,
            Xoshiro256PlusPlus::seed_from_u64(42),
            config,
            constraints,
        )
        .unwrap()
    }

    #[test]
    fn artificial_barrier_is_excluded_from_production_area() {
        let sep = separator(
            vec![
                (item(0, 20., 100., RotationRange::None), 1),
                (item(1, 10., 10., RotationRange::None), 100),
            ],
            true,
        );
        let bound = strip_width_lower_bound(&sep);
        assert!(
            (bound - 100.).abs() < 0.01,
            "内部挡板不能提高生产零件面积界：{bound}"
        );
    }

    #[test]
    fn width_bound_uses_only_rotations_that_fit_strip_height() {
        let sep = separator(
            vec![(
                item(
                    0,
                    120.,
                    20.,
                    RotationRange::Discrete(vec![0., std::f32::consts::FRAC_PI_2]),
                ),
                1,
            )],
            false,
        );
        let bound = strip_width_lower_bound(&sep);
        assert!(
            (bound - 120.).abs() < 0.01,
            "高度超过 100 的旋转不可把必要宽度降到 20：{bound}"
        );
    }
}
