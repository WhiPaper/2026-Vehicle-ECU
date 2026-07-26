import pytest
from pytest_embedded import Dut


@pytest.mark.parametrize("target", ["esp32"], indirect=True)
def test_ros_bridge(dut: Dut) -> None:
    dut.run_all_single_board_cases()
