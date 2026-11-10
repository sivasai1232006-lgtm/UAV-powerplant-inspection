from pathlib import Path

import yaml


def test_nav2_smac_planner_configs_use_hybrid_a_star():
    config_directory = Path(__file__).resolve().parents[1] / 'config'
    planner_configs = sorted(
        config_directory.glob('nav2_planner*.yaml')
    )

    assert planner_configs
    for config_path in planner_configs:
        config = yaml.safe_load(
            config_path.read_text(encoding='utf-8')
        )
        planner_parameters = next(iter(config.values()))['ros__parameters']
        hybrid_parameters = planner_parameters['GridBased']
        assert hybrid_parameters['plugin'] == (
            'nav2_smac_planner/SmacPlannerHybrid'
        ), config_path.name
        assert hybrid_parameters['motion_model_for_search'] == 'DUBIN'
        assert hybrid_parameters['angle_quantization_bins'] == 72
        assert hybrid_parameters['minimum_turning_radius'] == 0.4
        assert hybrid_parameters['tolerance'] == 1.0
