import json
from pathlib import Path
import carla
client=carla.Client('172.21.16.1',2000);client.set_timeout(20)
world=client.get_world();settings=world.get_settings()
actors=[{'id':a.id,'type':a.type_id} for a in world.get_actors() if a.type_id.startswith(('vehicle.','sensor.'))]
result={'mode':'READ_ONLY_NO_TICK','remaining_vehicle_or_sensor_actors':actors,'no_remaining_vehicle_or_sensor_actors':not actors,'synchronous_mode':settings.synchronous_mode,'fixed_delta_seconds':settings.fixed_delta_seconds}
Path('artifacts/carla_estimation/cleanup_after_legacy.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
