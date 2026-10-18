from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from calibration.iris_dynamics import STOCK  # noqa: E402
from calibration.search import candidate_count, neighbors  # noqa: E402

def test_stock_neighbors_move_one_field_and_clamp_mass():
    found = neighbors(STOCK)
    assert candidate_count(STOCK) == 1 + len(found)
    masses = sorted({d.mass_kg for d in found})
    assert 0.75 in masses
    assert 2.0 in masses  # 3.0 clamped to the 2.0 bound
    assert 3.0 not in masses
    for d in found:
        changed = [name for name in d.__dataclass_fields__ if getattr(d, name) != getattr(STOCK, name)]
        assert len(changed) == 1
