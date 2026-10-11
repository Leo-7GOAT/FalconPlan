from types import SimpleNamespace

import numpy as np
import pytest

from demos.carla_record_gallery import CameraTimeline


def test_missing_camera_after_gpu_warmup_fails_instead_of_recording_a_still():
    timeline=CameraTimeline(start_frame=100)
    timeline.pace(104,timeout=.005)
    with pytest.raises(TimeoutError):
        timeline.pace(105,timeout=.005)


def test_duplicate_camera_packet_cannot_corrupt_the_timeline():
    timeline=CameraTimeline()
    packet=SimpleNamespace(frame=10,timestamp=.5,width=2,height=2,raw_data=np.zeros((2,2,4),dtype=np.uint8).tobytes())
    timeline.receive(packet)
    with pytest.raises(ValueError):timeline.receive(packet)
    assert len(timeline.images)==1 and timeline.last_frame==10


def test_empty_or_startup_only_recording_cannot_be_published(tmp_path):
    for entries in ([],[(10,0.,np.zeros((2,2,3),dtype=np.uint8))]):
        timeline=CameraTimeline();timeline.images=entries
        with pytest.raises(RuntimeError,match='coverage'):
            timeline.write(tmp_path,0.,20.)
        assert not (tmp_path/'simulation.gif').exists()
