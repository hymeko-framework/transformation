//! Static obstacles are projected from typed canonical nodes, not naming conventions.
use std::path::Path;

use hymeko::module_store::{module_store::ModuleStore, source_provider::MemProvider};
use hymeko::util::real_parser::RealParser;
use hymeko_formats::gazebo::generate_gazebo_world;

fn render(source: &str) -> String {
    let provider = MemProvider::default().with_file("/obstacles.hymeko", source);
    let mut store = ModuleStore::new(provider, RealParser);
    let program = store.compile(Path::new("/obstacles.hymeko")).unwrap();
    generate_gazebo_world(&program.ir, &store.it, "unused", "obstacle_test")
}

#[test]
fn typed_obstacles_keep_distinct_positions_and_collision_geometry() {
    let world = render(
        r#"obstacle_scene {}
        gazebo_static_box {}
        first: gazebo_static_box {
            x 2; y 3; z 0.5; size_x 1; size_y 1; size_z 1;
        }
        second: gazebo_static_box {
            x -4; y 5; z 1; size_x 0.5; size_y 2; size_z 2;
        }
        untyped_decoy { x 99; y 99; z 99; size_x 1; size_y 1; size_z 1; }
        "#,
    );
    assert_eq!(world.matches("<model name=").count(), 3); // ground + two obstacles
    assert_eq!(world.matches("<static>true</static>").count(), 3);
    assert!(world.contains("<model name=\"first\">"));
    assert!(world.contains("<pose>2 3 0.5 0 0 0</pose>"));
    assert!(world.contains("<model name=\"second\">"));
    assert!(world.contains("<pose>-4 5 1 0 0 0</pose>"));
    assert_eq!(world.matches("<box><size>1 1 1</size></box>").count(), 2);
    assert_eq!(world.matches("<box><size>0.5 2 2</size></box>").count(), 2);
    assert!(!world.contains("untyped_decoy"));
    assert!(!world.contains("{{"));
}

#[test]
fn scenes_without_obstacles_still_contain_only_the_ground_model() {
    let world = render("empty_scene {}\ncontext {}\n");
    assert_eq!(world.matches("<model name=").count(), 1);
    assert!(world.contains("<model name=\"ground_plane\">"));
    assert!(world.contains("<world name=\"obstacle_test\">"));
}
