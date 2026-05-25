"""Tkinter GUI — program-first teach workflow."""
import tkinter as tk
from tkinter import messagebox

from ..core.arm import ArmMode
from ..core.gripper import GripperState

# ── palette ──────────────────────────────────────────────────────────────────
BG     = "#111214"
PANEL  = "#1a1c20"
BTN    = "#222428"
ACCENT = "#5aab57"
WARN   = "#c8a44a"
DANGER = "#c0392b"
TEXT   = "#dedad4"
MUTED  = "#6b6860"


def _btn(parent, text, cmd, fg=TEXT, width=None, **kw):
    kwargs = dict(text=text, command=cmd, bg=BTN, fg=fg,
                  activebackground=PANEL, activeforeground=fg,
                  relief="flat", font=("Courier", 10, "bold"),
                  padx=10, pady=6, cursor="hand2", **kw)
    if width:
        kwargs["width"] = width
    return tk.Button(parent, **kwargs)


class ArmGUI:
    def __init__(self, arm):
        self.arm     = arm
        self.gripper = arm.gripper

        self.root = tk.Tk()
        self.root.title("Arm Controller")
        self.root.configure(bg=BG)
        self.root.resizable(False, False)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self._mode_var      = tk.StringVar(value="IDLE")
        self._grip_var      = tk.StringVar(value="UNKNOWN")
        self._angle_vars    = [tk.StringVar(value="  0.0°") for _ in range(6)]
        self._steps_hdr_var = tk.StringVar(value="STEPS")
        self._active_prog   = None  # Program object currently selected

        self._build_ui()
        self._register_callbacks()
        self._refresh_all()

    # ─────────────────────────────────────────────────────────────── build UI

    def _build_ui(self):
        r = self.root

        tk.Label(r, text="ROBOT ARM CONTROLLER", bg=BG, fg=ACCENT,
                 font=("Courier", 13, "bold")).pack(pady=(14, 0))

        # ── status ───────────────────────────────────────────────────────────
        sf = self._panel(r)
        sf.pack(fill="x", padx=12, pady=8)

        top = tk.Frame(sf, bg=PANEL)
        top.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(top, text="MODE:", bg=PANEL, fg=MUTED,
                 font=("Courier", 9)).pack(side="left")
        self._mode_lbl = tk.Label(top, textvariable=self._mode_var,
                                   bg=PANEL, fg=ACCENT,
                                   font=("Courier", 10, "bold"))
        self._mode_lbl.pack(side="left", padx=6)

        af = tk.Frame(sf, bg=PANEL)
        af.pack(fill="x", padx=10, pady=(0, 10))
        names = ["J1 Base", "J2 Shoulder", "J3 Elbow",
                 "J4 WristP", "J5 WristR", "J6 Grip"]
        for i, (n, v) in enumerate(zip(names, self._angle_vars)):
            row, col = divmod(i, 3)
            tk.Label(af, text=n, bg=PANEL, fg=MUTED,
                     font=("Courier", 8), anchor="e", width=10
                     ).grid(row=row, column=col * 2, sticky="e", padx=(4, 2))
            tk.Label(af, textvariable=v, bg=PANEL, fg=TEXT,
                     font=("Courier", 9, "bold"), anchor="w", width=8
                     ).grid(row=row, column=col * 2 + 1, sticky="w")

        # ── freedrive + gripper ───────────────────────────────────────────────
        mid = tk.Frame(r, bg=BG)
        mid.pack(fill="x", padx=12, pady=2)

        fdf = self._panel(mid)
        fdf.pack(side="left", fill="both", expand=True, padx=(0, 5))
        tk.Label(fdf, text="FREEDRIVE", bg=PANEL, fg=MUTED,
                 font=("Courier", 8)).pack(pady=(8, 4))
        self._fd_btn = _btn(fdf, "ENABLE", self._toggle_freedrive, fg=TEXT, width=12)
        self._fd_btn.pack(padx=12, pady=(0, 10), fill="x")

        grf = self._panel(mid)
        grf.pack(side="left", fill="both", expand=True, padx=(5, 0))
        tk.Label(grf, text="GRIPPER", bg=PANEL, fg=MUTED,
                 font=("Courier", 8)).pack(pady=(8, 4))
        gr = tk.Frame(grf, bg=PANEL)
        gr.pack()
        _btn(gr, "OPEN",  self._gripper_open,  fg=ACCENT).pack(side="left", padx=3)
        _btn(gr, "CLOSE", self._gripper_close, fg=TEXT  ).pack(side="left", padx=3)
        self._grip_lbl = tk.Label(grf, textvariable=self._grip_var,
                                   bg=PANEL, fg=MUTED,
                                   font=("Courier", 9, "bold"))
        self._grip_lbl.pack(pady=(6, 10))

        # ── programs ─────────────────────────────────────────────────────────
        pgf = self._panel(r)
        pgf.pack(fill="x", padx=12, pady=8)
        self._section(pgf, "PROGRAMS")

        # new-program row
        pnr = tk.Frame(pgf, bg=PANEL)
        pnr.pack(fill="x", padx=10, pady=(0, 4))
        tk.Label(pnr, text="Name:", bg=PANEL, fg=TEXT,
                 font=("Courier", 9)).pack(side="left")
        self._prog_entry = tk.Entry(pnr, bg=BTN, fg=TEXT, insertbackground=TEXT,
                                     font=("Courier", 10), relief="flat", width=16)
        self._prog_entry.pack(side="left", padx=6)
        self._prog_entry.bind("<Return>", lambda _: self._new_program())
        _btn(pnr, "NEW", self._new_program, fg=ACCENT).pack(side="left")

        self._prog_box = tk.Listbox(pgf, bg=BTN, fg=TEXT,
                                     selectbackground=ACCENT, selectforeground=PANEL,
                                     font=("Courier", 10), relief="flat", height=4,
                                     activestyle="none", exportselection=False)
        self._prog_box.pack(fill="x", padx=10)
        self._prog_box.bind("<<ListboxSelect>>", self._on_prog_select)

        pa = tk.Frame(pgf, bg=PANEL)
        pa.pack(fill="x", padx=10, pady=(4, 10))
        _btn(pa, "RUN",     self._run_program,    fg=ACCENT ).pack(side="left", padx=(0, 4))
        _btn(pa, "STOP",    self._stop_program,   fg=DANGER ).pack(side="left", padx=(0, 4))
        _btn(pa, "DELETE",  self._delete_program, fg=DANGER ).pack(side="right")

        # ── program steps ─────────────────────────────────────────────────────
        stf = self._panel(r)
        stf.pack(fill="x", padx=12, pady=8)

        shdr = tk.Frame(stf, bg=PANEL)
        shdr.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(shdr, textvariable=self._steps_hdr_var, bg=PANEL, fg=MUTED,
                 font=("Courier", 8)).pack(side="left")

        self._steps_box = tk.Listbox(stf, bg=BTN, fg=TEXT,
                                      selectbackground=ACCENT, selectforeground=PANEL,
                                      font=("Courier", 10), relief="flat", height=4,
                                      activestyle="none")
        self._steps_box.pack(fill="x", padx=10)

        sa = tk.Frame(stf, bg=PANEL)
        sa.pack(fill="x", padx=10, pady=(4, 10))
        _btn(sa, "▲",           self._step_up,     fg=TEXT  ).pack(side="left", padx=(0, 4))
        _btn(sa, "▼",           self._step_down,   fg=TEXT  ).pack(side="left", padx=(0, 4))
        _btn(sa, "REMOVE STEP", self._remove_step, fg=DANGER).pack(side="left")

        # ── waypoints ─────────────────────────────────────────────────────────
        wpf = self._panel(r)
        wpf.pack(fill="x", padx=12, pady=8)
        self._section(wpf, "WAYPOINTS")

        tr = tk.Frame(wpf, bg=PANEL)
        tr.pack(fill="x", padx=10, pady=(0, 6))
        tk.Label(tr, text="Name:", bg=PANEL, fg=TEXT,
                 font=("Courier", 9)).pack(side="left")
        self._wp_entry = tk.Entry(tr, bg=BTN, fg=TEXT, insertbackground=TEXT,
                                   font=("Courier", 10), relief="flat", width=16)
        self._wp_entry.pack(side="left", padx=6)
        self._wp_entry.bind("<Return>", lambda _: self._teach_waypoint())
        _btn(tr, "TEACH", self._teach_waypoint, fg=ACCENT).pack(side="left")

        self._wp_box = tk.Listbox(wpf, bg=BTN, fg=TEXT,
                                   selectbackground=ACCENT, selectforeground=PANEL,
                                   font=("Courier", 10), relief="flat", height=5,
                                   activestyle="none", exportselection=False)
        self._wp_box.pack(fill="x", padx=10)

        wa = tk.Frame(wpf, bg=PANEL)
        wa.pack(fill="x", padx=10, pady=(4, 10))
        _btn(wa, "+ ADD TO PROGRAM", self._add_to_program, fg=ACCENT).pack(side="left", padx=(0, 6))
        _btn(wa, "DELETE",           self._delete_waypoint, fg=DANGER).pack(side="left")

        tk.Frame(r, bg=BG, height=10).pack()

    # ─────────────────────────────────────────────────────────────── helpers

    def _panel(self, parent):
        return tk.Frame(parent, bg=PANEL)

    def _section(self, parent, text):
        tk.Label(parent, text=text, bg=PANEL, fg=MUTED,
                 font=("Courier", 8)).pack(anchor="w", padx=10, pady=(8, 4))

    def _selected_program(self):
        sel = self._prog_box.curselection()
        if not sel:
            return None
        name = self._prog_box.get(sel[0])
        return self.arm.store.get_program(name)

    def _selected_step_idx(self):
        sel = self._steps_box.curselection()
        return sel[0] if sel else None

    def _selected_waypoint(self):
        sel = self._wp_box.curselection()
        return self._wp_box.get(sel[0]) if sel else None

    # ──────────────────────────────────── callbacks (called from bg threads)

    def _register_callbacks(self):
        self.arm.on_angles_changed    = self._on_angles
        self.arm.on_mode_changed      = self._on_mode
        self.gripper.on_state_changed = self._on_gripper

    def _on_angles(self, angles):
        def _u():
            for var, a in zip(self._angle_vars, angles):
                var.set(f"{a:+.1f}°")
        self.root.after(0, _u)

    def _on_mode(self, mode):
        colors = {ArmMode.IDLE: TEXT, ArmMode.RUNNING: ACCENT,
                  ArmMode.FREEDRIVE: WARN, ArmMode.FAULT: DANGER}
        def _u():
            self._mode_var.set(mode.upper())
            self._mode_lbl.config(fg=colors.get(mode, TEXT))
            if mode == ArmMode.FREEDRIVE:
                self._fd_btn.config(text="DISABLE", fg=WARN)
            else:
                self._fd_btn.config(text="ENABLE", fg=TEXT)
        self.root.after(0, _u)

    def _on_gripper(self, state):
        colors = {GripperState.OPEN:    ACCENT,
                  GripperState.CLOSING: WARN,
                  GripperState.HOLDING: DANGER,
                  GripperState.UNKNOWN: MUTED}
        def _u():
            self._grip_var.set(state.value.upper())
            self._grip_lbl.config(fg=colors.get(state, MUTED))
        self.root.after(0, _u)

    # ───────────────────────────────────────────────────────── button handlers

    def _toggle_freedrive(self):
        self.arm.set_freedrive(self.arm.mode != ArmMode.FREEDRIVE)

    def _gripper_open(self):  self.gripper.open()
    def _gripper_close(self): self.gripper.close()

    # programs

    def _on_prog_select(self, _event=None):
        prog = self._selected_program()
        if prog:
            self._active_prog = prog
            self._steps_hdr_var.set(f"STEPS  ←  {prog.name}")
        else:
            self._active_prog = None
            self._steps_hdr_var.set("STEPS")
        self._refresh_steps()

    def _new_program(self):
        name = self._prog_entry.get().strip()
        if not name:
            messagebox.showwarning("New Program", "Enter a program name first.")
            return
        if self.arm.store.get_program(name):
            messagebox.showwarning("New Program", f'"{name}" already exists.')
            return
        self.arm.store.add_program(name)
        self._prog_entry.delete(0, "end")
        self._refresh_programs()
        # auto-select the new program
        items = self._prog_box.get(0, "end")
        if name in items:
            idx = list(items).index(name)
            self._prog_box.selection_set(idx)
            self._on_prog_select()

    def _delete_program(self):
        prog = self._selected_program()
        if not prog:
            return
        self.arm.store.remove_program(prog.name)
        self._active_prog = None
        self._steps_hdr_var.set("STEPS")
        self._refresh_programs()
        self._refresh_steps()

    def _run_program(self):
        prog = self._selected_program()
        if not prog:
            messagebox.showinfo("Run", "Select a program first.")
            return
        self.arm.run_program(prog.name)

    def _stop_program(self):
        self.arm.stop_program()

    # steps

    def _step_up(self):
        prog = self._active_prog
        idx  = self._selected_step_idx()
        if prog is None or idx is None or idx == 0:
            return
        prog.move_step(idx, idx - 1)
        self.arm.store.save_all()
        self._refresh_steps()
        self._steps_box.selection_set(idx - 1)

    def _step_down(self):
        prog = self._active_prog
        idx  = self._selected_step_idx()
        if prog is None or idx is None or idx >= len(prog.steps) - 1:
            return
        prog.move_step(idx, idx + 1)
        self.arm.store.save_all()
        self._refresh_steps()
        self._steps_box.selection_set(idx + 1)

    def _remove_step(self):
        prog = self._active_prog
        idx  = self._selected_step_idx()
        if prog is None or idx is None:
            return
        prog.remove_step(idx)
        self.arm.store.save_all()
        self._refresh_steps()

    def _add_to_program(self):
        prog    = self._active_prog
        wp_name = self._selected_waypoint()
        if prog is None:
            messagebox.showinfo("Add Step", "Select a program first.")
            return
        if wp_name is None:
            messagebox.showinfo("Add Step", "Select a waypoint first.")
            return
        prog.add_step(wp_name, speed_pct=40)
        self.arm.store.save_all()
        self._refresh_steps()

    # waypoints

    def _teach_waypoint(self):
        name = self._wp_entry.get().strip()
        if not name:
            messagebox.showwarning("Teach", "Enter a waypoint name first.")
            return
        self.arm.save_waypoint(name)
        self._wp_entry.delete(0, "end")
        self._refresh_waypoints()

    def _delete_waypoint(self):
        name = self._selected_waypoint()
        if name is None:
            return
        self.arm.delete_waypoint(name)
        self._refresh_waypoints()

    # ──────────────────────────────────────────────────────── refresh helpers

    def _refresh_all(self):
        self._refresh_programs()
        self._refresh_steps()
        self._refresh_waypoints()

    def _refresh_programs(self):
        self._prog_box.delete(0, "end")
        for pg in self.arm.store.list_programs():
            self._prog_box.insert("end", pg.name)

    def _refresh_steps(self):
        self._steps_box.delete(0, "end")
        prog = self._active_prog
        if prog is None:
            return
        # Re-read from store in case it was modified
        prog = self.arm.store.get_program(prog.name)
        if prog is None:
            return
        self._active_prog = prog
        for i, step in enumerate(prog.steps):
            self._steps_box.insert("end", f"{i+1}.  {step.waypoint_name}")

    def _refresh_waypoints(self):
        self._wp_box.delete(0, "end")
        for wp in self.arm.list_waypoints():
            self._wp_box.insert("end", wp.name)

    # ─────────────────────────────────────────────────────────────────── run

    def _on_close(self):
        self.arm.set_freedrive(False)
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def run_gui(arm):
    ArmGUI(arm).run()
