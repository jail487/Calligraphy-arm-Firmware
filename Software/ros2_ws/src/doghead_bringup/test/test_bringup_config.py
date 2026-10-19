import os
import yaml
import pytest


def test_safety_limits_yaml_defaults():
    config_path = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'safety_limits.yaml'
    )
    assert os.path.exists(config_path), "safety_limits.yaml file missing!"

    with open(config_path, 'r') as f:
        data = yaml.safe_load(f)

    # 1. Verify driver safety parameter defaults
    driver_params = data.get('doghead_driver_node', {}).get('ros__parameters', {})
    assert driver_params.get('shadow_mode') is True, "shadow_mode MUST be true by default!"
    assert driver_params.get('actuator_enabled') is False, "actuator_enabled MUST be false by default!"
    assert driver_params.get('protocol_version') == 1, "protocol_version MUST be 1!"

    # 2. Verify HIL scenario runner safety parameter defaults
    hil_params = data.get('hil_scenario_runner_node', {}).get('ros__parameters', {})
    assert hil_params.get('actuator_enabled') is False, "HIL scenario runner actuator_enabled MUST be false by default!"
