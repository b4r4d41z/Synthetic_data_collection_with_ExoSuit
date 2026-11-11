"""Fixed scene/config registry shared by shell startup, Kit and ROS metadata."""
from pathlib import Path
import json
import sys

SCENARIOS = {
    'v1_manipulation_one_side_right': ('stage_sorting_v1.usd', 'sorting_scene.json'),
    'v1_manipulation_ambidextrous': ('stage_manipulation.usda', 'sorting_scene.json'),
    'sorting_experimental': ('stage_sorting_v3.usda', 'sorting_scene_v3.json'),
}
DEFAULT = 'sorting_experimental'


def scenario(name, repository=None):
    if name not in SCENARIOS:
        raise ValueError(f'Unknown scenario {name!r}; choose '+', '.join(SCENARIOS))
    stage, config = SCENARIOS[name]
    relative = Path('models/stages')/name
    if repository is None:
        repository = next((p for p in Path(__file__).resolve().parents
                           if (p/relative/config).is_file()), None)
    if repository is None:
        from ament_index_python.packages import get_package_share_directory
        config_path = Path(get_package_share_directory('isaac_eff_control'))/'scenarios'/name/config
        stage_path = None  # Robot/stage assets still belong to the checkout.
    else:
        stage_path = Path(repository).resolve()/relative/stage
        config_path = Path(repository).resolve()/relative/config
        if not stage_path.is_file():raise ValueError(f'Missing scenario scene: {stage_path}')
    if not config_path.is_file():raise ValueError(f'Missing scenario configuration: {config_path}')
    data = json.loads(config_path.read_text())
    if data.get('scene') != str(relative/stage):
        raise ValueError(f'Scenario/configuration mismatch: {config_path}')
    if repository is not None:
        pool = Path(repository)/data['provenance']['fruit_pool_asset']
        if not pool.is_file():raise ValueError(f'Missing scenario pool: {pool}')
        if name == 'v1_manipulation_ambidextrous':
            for dependency in ('sorting_experimental/things_pool.usda', 'v1_manipulation_one_side_right/stage_sorting_v1.usd'):
                if not (Path(repository)/'models/stages'/dependency).is_file():
                    raise ValueError(f'Missing scene dependency: {dependency}')
        if name == 'sorting_experimental':
            dependencies = ('sorting_experimental/stage_sorting_v2.usda',
                            'v1_manipulation_one_side_right/stage_sorting_v1.usd', 'v1_manipulation_one_side_right/fruit_pool.usda')
            for dependency in dependencies:
                required = Path(repository)/'models/stages'/dependency
                if not required.is_file():raise ValueError(f'Missing scene dependency: {required}')
    return dict(name=name, scene=str(relative/stage), stage=stage_path, config=config_path)


def scenario_for_stage(path, repository):
    """Exact paths only; filenames alone cannot select a configuration."""
    path = Path(path).resolve()
    for name, (stage, _) in SCENARIOS.items():
        entry = scenario(name, repository)
        if path == entry['stage']:
            return entry
    # V2 remains an auxiliary experimental physics layer over legacy fruit
    # placement, exactly as before; it is not a selectable scenario.
    if path == (Path(repository)/'models/stages/sorting_experimental/stage_sorting_v2.usda').resolve():
        return scenario('v1_manipulation_one_side_right', repository)
    return None


if __name__ == '__main__':
    try:
        selected = scenario(sys.argv[1], sys.argv[2])
        print(selected['stage']); print(selected['config'])
    except (ValueError, IndexError) as error:
        raise SystemExit(str(error))
