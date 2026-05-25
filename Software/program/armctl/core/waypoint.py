import json, time
from dataclasses import dataclass, field, asdict
from typing import List, Optional
from pathlib import Path

@dataclass
class Waypoint:
    name: str; positions: List[int]; created_at: float = field(default_factory=time.time); description: str = ""
    def to_dict(self): return asdict(self)
    @classmethod
    def from_dict(cls, data):
        d = dict(data)
        if "angles" in d and "positions" not in d:
            d["positions"] = [round(a * 4096.0 / 360.0 + 2048) for a in d.pop("angles")]
        elif "angles" in d:
            d.pop("angles")
        return cls(**d)

@dataclass
class ProgramStep:
    waypoint_name: str; speed_pct: int = 50; delay_after: float = 0.0; blend_radius: float = 0.0
    def to_dict(self): return asdict(self)
    @classmethod
    def from_dict(cls, data): return cls(**data)

@dataclass
class Program:
    name: str; steps: List[ProgramStep] = field(default_factory=list)
    loop: bool = False; created_at: float = field(default_factory=time.time); description: str = ""
    def add_step(self, wp_name, speed_pct=50, delay_after=0.0):
        self.steps.append(ProgramStep(wp_name, speed_pct, delay_after))
    def remove_step(self, idx):
        if 0 <= idx < len(self.steps): self.steps.pop(idx)
    def move_step(self, from_idx, to_idx):
        if 0 <= from_idx < len(self.steps) and 0 <= to_idx < len(self.steps):
            self.steps.insert(to_idx, self.steps.pop(from_idx))
    def to_dict(self):
        return {"name":self.name,"steps":[s.to_dict() for s in self.steps],"loop":self.loop,"created_at":self.created_at,"description":self.description}
    @classmethod
    def from_dict(cls, data):
        return cls(name=data["name"],steps=[ProgramStep.from_dict(s) for s in data.get("steps",[])],
                   loop=data.get("loop",False),created_at=data.get("created_at",time.time()),description=data.get("description",""))

class WaypointStore:
    def __init__(self, save_dir="programs"):
        self.save_dir = Path(save_dir); self.save_dir.mkdir(exist_ok=True)
        self.waypoints = {}; self.programs = {}
        self._wf = self.save_dir/"waypoints.json"; self._pf = self.save_dir/"programs.json"
    def add_waypoint(self, name, positions, description=""):
        wp = Waypoint(name=name, positions=[round(p) for p in positions], description=description)
        self.waypoints[name] = wp; self._save_wp(); return wp
    def remove_waypoint(self, name):
        if name in self.waypoints: del self.waypoints[name]; self._save_wp()
    def get_waypoint(self, name): return self.waypoints.get(name)
    def list_waypoints(self): return list(self.waypoints.values())
    def add_program(self, name, description=""):
        prog = Program(name=name, description=description); self.programs[name] = prog; self._save_pg(); return prog
    def remove_program(self, name):
        if name in self.programs: del self.programs[name]; self._save_pg()
    def get_program(self, name): return self.programs.get(name)
    def list_programs(self): return list(self.programs.values())
    def save_all(self): self._save_wp(); self._save_pg()
    def load_all(self): self._load_wp(); self._load_pg()
    def _save_wp(self): self._wf.write_text(json.dumps({n:w.to_dict() for n,w in self.waypoints.items()}, indent=2))
    def _load_wp(self):
        if self._wf.exists(): self.waypoints = {n:Waypoint.from_dict(w) for n,w in json.loads(self._wf.read_text()).items()}
    def _save_pg(self): self._pf.write_text(json.dumps({n:p.to_dict() for n,p in self.programs.items()}, indent=2))
    def _load_pg(self):
        if self._pf.exists(): self.programs = {n:Program.from_dict(p) for n,p in json.loads(self._pf.read_text()).items()}
