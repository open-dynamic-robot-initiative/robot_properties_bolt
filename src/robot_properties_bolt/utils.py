try:
    # use standard Python importlib if available (>Python3.7)
    import importlib.resources as importlib_resources
except ImportError:
    import importlib_resources


def find_paths(robot_name, robot_family="bolt"):
    with importlib_resources.path(__package__, "utils.py") as p:
        package_dir = p.parent.absolute()

    resources_dir = package_dir / ("robot_properties_" + robot_family)
    dgm_yaml_path = (
        resources_dir
        / "dynamic_graph_manager"
        / ("dgm_parameters_" + robot_name + ".yaml")
    )
    urdf_path = resources_dir / (robot_name + ".urdf")
    simu_urdf_path = resources_dir / "bolt_passive_ankle.urdf"
    srdf_path = resources_dir / "srdf" / (robot_family + ".srdf")
    ctrl_path = resources_dir / "impedance_ctrl.yaml"

    paths = {
        "package": str(package_dir),
        "resources": str(resources_dir),
        "dgm_yaml": str(dgm_yaml_path),
        "srdf": str(srdf_path),
        "urdf": str(urdf_path),
        "simu_urdf": str(simu_urdf_path),
        "imp_ctrl_yaml": str(ctrl_path),
    }

    return paths
