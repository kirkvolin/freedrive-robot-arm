"""
Robot Arm Simulator — Waypoint Teach & Playback
=================================================
"""

import sys, math, time
import pygame
import numpy as np
from arm_controller import (ArmController, WaypointManager, ProgramPlayer,
                            forward_kinematics, inverse_kinematics)

# ============================================================
# COLORS
# ============================================================
BG           = (13, 17, 23)
PANEL_BG     = (22, 27, 34)
BORDER       = (48, 54, 61)
TEXT         = (201, 209, 217)
TEXT_DIM     = (106, 115, 125)
GRID_COLOR   = (26, 35, 50)
WP_LIST_BG   = (16, 20, 28)
WP_HIGHLIGHT = (35, 45, 60)
WP_ACTIVE    = (40, 60, 90)

JOINT_COLORS = [
    (231, 76, 60), (230, 126, 34), (241, 196, 15),
    (46, 204, 113), (52, 152, 219), (155, 89, 182),
]
LINK_COLOR   = (120, 144, 168)
AXIS_X, AXIS_Y, AXIS_Z = (231,76,60), (46,204,113), (52,152,219)
EE_COLOR     = (255, 100, 100)
TARGET_COLOR = (255, 255, 100)

STATUS_READY = (46, 204, 113)
STATUS_PLAY  = (52, 152, 219)
STATUS_FREE  = (241, 196, 15)
STATUS_PAUSE = (230, 126, 34)
WP_GHOST     = (80, 180, 255, 60)

# ============================================================
# 3D PROJECTION
# ============================================================

def project_3d(x, y, z, view_h, view_v, cx, cy, scale=1.1):
    rad_h, rad_v = math.radians(view_h), math.radians(view_v)
    x1 = x * math.cos(rad_h) - z * math.sin(rad_h)
    z1 = x * math.sin(rad_h) + z * math.cos(rad_h)
    y2 = y * math.cos(rad_v) - z1 * math.sin(rad_v)
    z2 = y * math.sin(rad_v) + z1 * math.cos(rad_v)
    perspective = 800
    factor = perspective / (perspective + z2)
    return cx + x1 * scale * factor, cy - y2 * scale * factor, z2, factor

def screen_to_world_ray(sx, sy, view_h, view_v, cx, cy, scale=1.1):
    rad_h, rad_v = math.radians(-view_h), math.radians(-view_v)
    nx, ny = (sx - cx) / scale, -(sy - cy) / scale
    y1 = ny * math.cos(rad_v); z1 = ny * math.sin(rad_v); x1 = nx
    return np.array([
        x1 * math.cos(rad_h) - z1 * math.sin(rad_h),
        y1,
        x1 * math.sin(rad_h) + z1 * math.cos(rad_h),
    ])

# ============================================================
# UI COMPONENTS
# ============================================================

class Slider:
    def __init__(self, x, y, w, h, min_val, max_val, value, color, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val; self.max_val = max_val
        self.value = value; self.color = color; self.label = label; self.dragging = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            hit = pygame.Rect(self.rect.x, self.rect.y + 8, self.rect.w, self.rect.h - 8)
            if hit.collidepoint(event.pos):
                self.dragging = True; self._update(event.pos[0]); return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1: self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging: self._update(event.pos[0]); return True
        return False

    def _update(self, mx):
        t = max(0, min(1, (mx - self.rect.x) / self.rect.w))
        self.value = self.min_val + t * (self.max_val - self.min_val)

    def draw(self, surface, font_sm, font_xs):
        surface.blit(font_sm.render(self.label, True, self.color), (self.rect.x, self.rect.y))
        val_s = font_sm.render(f"{self.value:.1f}\u00b0", True, self.color)
        surface.blit(val_s, (self.rect.right - val_s.get_width(), self.rect.y))
        track_y = self.rect.y + 22
        pygame.draw.rect(surface, BORDER, (self.rect.x, track_y, self.rect.w, 4), border_radius=2)
        t = (self.value - self.min_val) / (self.max_val - self.min_val)
        fw = int(self.rect.w * t)
        if fw > 0: pygame.draw.rect(surface, self.color, (self.rect.x, track_y, fw, 4), border_radius=2)
        tx = self.rect.x + fw
        pygame.draw.circle(surface, self.color, (tx, track_y + 2), 6)
        pygame.draw.circle(surface, (255,255,255), (tx, track_y + 2), 3)

class Button:
    def __init__(self, x, y, w, h, label, color, font):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label; self.color = color; self.font = font; self.hovered = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION: self.hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and self.rect.collidepoint(event.pos):
            return True
        return False

    def draw(self, surface, active=False, enabled=True):
        border = self.color if (active or self.hovered) else BORDER
        pygame.draw.rect(surface, PANEL_BG, self.rect, border_radius=4)
        pygame.draw.rect(surface, border, self.rect, 1, border_radius=4)
        tc = self.color if active else (TEXT_DIM if enabled else (60,60,60))
        ls = self.font.render(self.label, True, tc)
        surface.blit(ls, (self.rect.centerx - ls.get_width()//2, self.rect.centery - ls.get_height()//2))


# ============================================================
# MAIN APPLICATION
# ============================================================

class RobotArmApp:
    def __init__(self):
        pygame.init()
        self.width, self.height = 1200, 750
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("Robot Arm — Waypoint Teach & Playback")

        self.font_lg = pygame.font.SysFont("monospace", 16, bold=True)
        self.font_md = pygame.font.SysFont("monospace", 13)
        self.font_sm = pygame.font.SysFont("monospace", 11)
        self.font_xs = pygame.font.SysFont("monospace", 10)

        # Core
        self.controller = ArmController()
        self.wp_manager = WaypointManager()
        self.player = ProgramPlayer()
        self.controller.home_all()

        # View
        self.view_h, self.view_v = 35, 25
        self.orbiting = False
        self.orbit_start = (0, 0)
        self.orbit_start_angles = (0, 0)

        # IK drag
        self.ik_dragging = False
        self.ik_target = None

        # Layout
        self.viewport_rect = pygame.Rect(0, 0, 650, 750)
        self.panel_x = 660
        self.wp_scroll_offset = 0
        self.selected_wp_idx = -1

        self._build_ui()
        self.clock = pygame.time.Clock()
        self.running = True

    def _build_ui(self):
        sx, sy = self.panel_x, 75
        self.sliders = []
        for i, servo in enumerate(self.controller.servos):
            self.sliders.append(Slider(sx, sy + i*48, 340, 36,
                                       servo.min_angle, servo.max_angle,
                                       servo.get_angle(), JOINT_COLORS[i], servo.label))

        btn_y = sy + 6 * 48 + 8
        bw, bh, gap = 78, 28, 5

        self.btn_home = Button(sx, btn_y, bw, bh, "HOME", (52,152,219), self.font_sm)
        self.btn_free = Button(sx + (bw+gap), btn_y, bw, bh, "FREEDRV", STATUS_FREE, self.font_sm)
        self.btn_save = Button(sx + 2*(bw+gap), btn_y, bw, bh, "SAVE WP", (46,204,113), self.font_sm)
        self.btn_del  = Button(sx + 3*(bw+gap), btn_y, bw, bh, "DEL WP", (180,80,80), self.font_sm)

        btn_y2 = btn_y + bh + gap
        self.btn_play = Button(sx, btn_y2, bw, bh, "PLAY", STATUS_PLAY, self.font_sm)
        self.btn_stop = Button(sx + (bw+gap), btn_y2, bw, bh, "STOP", (231,76,60), self.font_sm)
        self.btn_loop = Button(sx + 2*(bw+gap), btn_y2, bw, bh, "LOOP", (155,89,182), self.font_sm)
        self.btn_clear = Button(sx + 3*(bw+gap), btn_y2, bw, bh, "CLEAR", TEXT_DIM, self.font_sm)

        self.wp_list_y = btn_y2 + bh + 18
        self.wp_list_h = self.height - self.wp_list_y - 50
        self.wp_list_rect = pygame.Rect(sx, self.wp_list_y, 340, self.wp_list_h)

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self._handle_events()
            self._update(dt)
            self._draw()
        pygame.quit()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: self.running = False; return

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: self.running = False
                elif event.key == pygame.K_h: self.controller.home_all(); self._sync_sliders()
                elif event.key == pygame.K_f: self._toggle_freedrive()
                elif event.key == pygame.K_s: self._save_waypoint()
                elif event.key == pygame.K_p: self._start_playback()
                elif event.key == pygame.K_x: self.player.stop()
                elif event.key == pygame.K_SPACE:
                    if self.player.paused: self.player.resume()
                    elif self.player.playing: self.player.pause()
                elif event.key == pygame.K_d or event.key == pygame.K_DELETE: self._delete_selected_wp()
                elif event.key == pygame.K_l: pass  # loop toggle handled by button

            # Buttons
            if self.btn_home.handle_event(event):
                self.controller.home_all(); self._sync_sliders()
            if self.btn_free.handle_event(event): self._toggle_freedrive()
            if self.btn_save.handle_event(event): self._save_waypoint()
            if self.btn_del.handle_event(event): self._delete_selected_wp()
            if self.btn_play.handle_event(event): self._start_playback()
            if self.btn_stop.handle_event(event): self.player.stop()
            if self.btn_loop.handle_event(event): pass  # toggled in _start_playback
            if self.btn_clear.handle_event(event): self.wp_manager.clear_all(); self.selected_wp_idx = -1

            # Waypoint list click
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.wp_list_rect.collidepoint(event.pos):
                    row = (event.pos[1] - self.wp_list_y - self.wp_scroll_offset) // 24
                    if 0 <= row < self.wp_manager.program_length:
                        self.selected_wp_idx = row
                    continue

            # Waypoint list scroll
            if event.type == pygame.MOUSEWHEEL and self.wp_list_rect.collidepoint(pygame.mouse.get_pos()):
                self.wp_scroll_offset += event.y * 24
                max_scroll = max(0, self.wp_manager.program_length * 24 - self.wp_list_h)
                self.wp_scroll_offset = max(-max_scroll, min(0, self.wp_scroll_offset))
                continue

            # Sliders (disabled during playback)
            if not self.player.playing:
                for i, slider in enumerate(self.sliders):
                    if slider.handle_event(event):
                        self.controller.set_joint_angle(i, slider.value)

            # Viewport interaction
            if event.type == pygame.MOUSEBUTTONDOWN and self.viewport_rect.collidepoint(event.pos):
                if event.button == 1:
                    if self._near_end_effector(event.pos):
                        self.ik_dragging = True
                        self.ik_target = self.controller.get_end_effector_pos().copy()
                    else:
                        self.orbiting = True
                        self.orbit_start = event.pos
                        self.orbit_start_angles = (self.view_h, self.view_v)
                elif event.button == 3:
                    self.ik_dragging = True
                    self.ik_target = self.controller.get_end_effector_pos().copy()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1: self.orbiting = False; self.ik_dragging = False
                elif event.button == 3: self.ik_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.orbiting:
                    dx = event.pos[0] - self.orbit_start[0]
                    dy = event.pos[1] - self.orbit_start[1]
                    self.view_h = self.orbit_start_angles[0] + dx * 0.5
                    self.view_v = max(-60, min(80, self.orbit_start_angles[1] + dy * 0.3))
                elif self.ik_dragging and not self.player.playing:
                    vp_cx = self.viewport_rect.centerx
                    vp_cy = self.viewport_rect.y + self.viewport_rect.h * 0.75
                    self.ik_target = screen_to_world_ray(event.pos[0], event.pos[1],
                                                         self.view_h, self.view_v, vp_cx, vp_cy)
                    self.controller.move_to_position(self.ik_target)
                    self._sync_sliders()

    def _toggle_freedrive(self):
        self.controller.freedrive = not self.controller.freedrive
        self.controller.set_freedrive(self.controller.freedrive)

    def _save_waypoint(self):
        wp = self.wp_manager.save_waypoint(self.controller.get_joint_angles())
        self.selected_wp_idx = self.wp_manager.program_length - 1

    def _delete_selected_wp(self):
        if 0 <= self.selected_wp_idx < self.wp_manager.program_length:
            name = self.wp_manager.program[self.selected_wp_idx]
            self.wp_manager.delete_waypoint(name)
            if self.selected_wp_idx >= self.wp_manager.program_length:
                self.selected_wp_idx = self.wp_manager.program_length - 1

    def _start_playback(self):
        if self.player.playing:
            if self.player.paused: self.player.resume()
            else: self.player.pause()
            return
        seq = self.wp_manager.get_program_sequence()
        if seq:
            self.player.start(seq, self.controller.get_joint_angles(), loop=False)

    def _sync_sliders(self):
        for i, s in enumerate(self.sliders):
            s.value = self.controller.get_joint_angles()[i]

    def _near_end_effector(self, mouse_pos, threshold=15):
        ee = self.controller.get_end_effector_pos()
        vp_cx = self.viewport_rect.centerx
        vp_cy = self.viewport_rect.y + self.viewport_rect.h * 0.75
        sx, sy, _, _ = project_3d(ee[0], ee[1], ee[2], self.view_h, self.view_v, vp_cx, vp_cy)
        return math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy) < threshold

    def _update(self, dt):
        if self.player.playing:
            angles = self.player.update(dt)
            if angles:
                self.controller.set_all_angles(angles)
                self._sync_sliders()

    # ============================================================
    # DRAWING
    # ============================================================

    def _draw(self):
        self.screen.fill(BG)
        self._draw_viewport()
        self._draw_panel()
        pygame.display.flip()

    def _draw_panel(self):
        panel_rect = pygame.Rect(self.panel_x - 10, 0, self.width - self.panel_x + 10, self.height)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, BORDER, (self.panel_x - 10, 0), (self.panel_x - 10, self.height))

        # Status
        if self.player.playing:
            if self.player.paused:
                sc, st_text = STATUS_PAUSE, "PAUSED"
            else:
                sc, st_text = STATUS_PLAY, f"PLAYING {self.player.current_step+1}/{len(self.player.sequence)}"
        elif self.controller.freedrive:
            sc, st_text = STATUS_FREE, "FREEDRIVE — move arm, press [S] to save"
        else:
            sc, st_text = STATUS_READY, "READY"

        pygame.draw.circle(self.screen, sc, (self.panel_x + 8, 20), 4)
        self.screen.blit(self.font_lg.render("WAYPOINT TEACH", True, TEXT), (self.panel_x + 20, 11))
        self.screen.blit(self.font_xs.render(st_text, True, sc), (self.panel_x + 20, 32))

        # TCP readout
        ee = self.controller.get_end_effector_pos()
        self.screen.blit(self.font_sm.render(
            f"TCP: ({ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f}) mm", True, TEXT_DIM),
            (self.panel_x + 20, 50))

        # Sliders
        for s in self.sliders: s.draw(self.screen, self.font_sm, self.font_xs)

        # Buttons row 1
        self.btn_home.draw(self.screen)
        self.btn_free.draw(self.screen, active=self.controller.freedrive)
        self.btn_save.draw(self.screen)
        self.btn_del.draw(self.screen, enabled=self.selected_wp_idx >= 0)

        # Buttons row 2
        has_wps = self.wp_manager.program_length > 0
        self.btn_play.draw(self.screen, active=self.player.playing and not self.player.paused, enabled=has_wps)
        self.btn_stop.draw(self.screen, enabled=self.player.playing)
        self.btn_loop.draw(self.screen, active=self.player.loop)
        self.btn_clear.draw(self.screen, enabled=has_wps)

        # Waypoint list header
        hdr_y = self.wp_list_y - 16
        self.screen.blit(self.font_xs.render(
            f"PROGRAM ({self.wp_manager.program_length} waypoints)", True, TEXT_DIM),
            (self.panel_x, hdr_y))

        # Waypoint list
        self._draw_waypoint_list()

        # Help
        help_lines = [
            "[H]ome [F]reedrive [S]ave WP [P]lay/Pause [X]Stop [D]elete",
            "Left-drag=orbit  Right-drag EE=IK  Scroll=WP list",
        ]
        for i, line in enumerate(help_lines):
            self.screen.blit(self.font_xs.render(line, True, TEXT_DIM),
                             (self.panel_x, self.height - 35 + i * 14))

    def _draw_waypoint_list(self):
        rect = self.wp_list_rect
        pygame.draw.rect(self.screen, WP_LIST_BG, rect)
        pygame.draw.rect(self.screen, BORDER, rect, 1)

        if self.wp_manager.program_length == 0:
            msg = self.font_sm.render("No waypoints yet", True, TEXT_DIM)
            self.screen.blit(msg, (rect.x + 10, rect.y + 10))
            sub = self.font_xs.render("Enable freedrive, move arm, press [S]", True, TEXT_DIM)
            self.screen.blit(sub, (rect.x + 10, rect.y + 28))
            return

        # Clip to list area
        clip = self.screen.get_clip()
        self.screen.set_clip(rect.inflate(-2, -2))

        row_h = 24
        for i, name in enumerate(self.wp_manager.program):
            wp = self.wp_manager.waypoints.get(name)
            if not wp: continue

            ry = rect.y + 2 + i * row_h + self.wp_scroll_offset
            if ry + row_h < rect.y or ry > rect.bottom: continue

            # Background highlight
            row_rect = pygame.Rect(rect.x + 2, ry, rect.w - 4, row_h - 2)
            if self.player.playing and i == self.player.current_step:
                pygame.draw.rect(self.screen, WP_ACTIVE, row_rect, border_radius=2)
            elif i == self.selected_wp_idx:
                pygame.draw.rect(self.screen, WP_HIGHLIGHT, row_rect, border_radius=2)

            # Index
            idx_s = self.font_xs.render(f"{i+1:2d}.", True, TEXT_DIM)
            self.screen.blit(idx_s, (rect.x + 6, ry + 4))

            # Name
            color = (52,152,219) if (self.player.playing and i == self.player.current_step) else TEXT
            name_s = self.font_sm.render(wp.name, True, color)
            self.screen.blit(name_s, (rect.x + 32, ry + 3))

            # Angles summary (compact)
            angles_str = "  ".join(f"{a:.0f}" for a in wp.angles[:5])
            ang_s = self.font_xs.render(angles_str, True, TEXT_DIM)
            self.screen.blit(ang_s, (rect.x + 110, ry + 6))

        self.screen.set_clip(clip)

        # Scroll indicator
        if self.wp_manager.program_length * row_h > rect.h:
            total_h = self.wp_manager.program_length * row_h
            bar_h = max(20, int(rect.h * rect.h / total_h))
            bar_y = rect.y + int(-self.wp_scroll_offset * (rect.h - bar_h) / (total_h - rect.h))
            pygame.draw.rect(self.screen, BORDER,
                             (rect.right - 6, bar_y, 4, bar_h), border_radius=2)

    def _draw_viewport(self):
        vp = self.viewport_rect
        pygame.draw.rect(self.screen, (8, 12, 18), vp)
        pygame.draw.rect(self.screen, BORDER, vp, 1)
        cx = vp.centerx
        cy = vp.y + vp.h * 0.75

        # Grid
        for i in range(-200, 201, 50):
            ax, ay, _, _ = project_3d(i, 0, -200, self.view_h, self.view_v, cx, cy)
            bx, by, _, _ = project_3d(i, 0, 200, self.view_h, self.view_v, cx, cy)
            if vp.clipline(ax, ay, bx, by):
                pygame.draw.line(self.screen, GRID_COLOR, (ax, ay), (bx, by), 1)
            ax, ay, _, _ = project_3d(-200, 0, i, self.view_h, self.view_v, cx, cy)
            bx, by, _, _ = project_3d(200, 0, i, self.view_h, self.view_v, cx, cy)
            if vp.clipline(ax, ay, bx, by):
                pygame.draw.line(self.screen, GRID_COLOR, (ax, ay), (bx, by), 1)

        # Axes
        ox, oy, _, _ = project_3d(0, 0, 0, self.view_h, self.view_v, cx, cy)
        for end, color, label in [((60,0,0),AXIS_X,"X"),((0,60,0),AXIS_Y,"Y"),((0,0,60),AXIS_Z,"Z")]:
            ex, ey, _, _ = project_3d(*end, self.view_h, self.view_v, cx, cy)
            pygame.draw.line(self.screen, color, (ox, oy), (ex, ey), 1)
            self.screen.blit(self.font_xs.render(label, True, color), (ex+3, ey-5))

        # Draw ghost arm at each waypoint
        self._draw_waypoint_ghosts(cx, cy)

        # Arm links and joints
        positions = self.controller.get_fk_positions()
        projected = []
        for pos in positions:
            sx, sy, sz, sf = project_3d(pos[0], pos[1], pos[2], self.view_h, self.view_v, cx, cy)
            projected.append((sx, sy, sz, sf))

        # Links
        for i in range(len(projected) - 1):
            p1, p2 = projected[i], projected[i+1]
            pygame.draw.line(self.screen, LINK_COLOR, (p1[0], p1[1]), (p2[0], p2[1]),
                             max(2, int(5 * p1[3])))

        # Gripper
        if len(projected) >= 5:
            ee = projected[-1]
            grip_open = (self.controller.get_joint_angles()[5] / 100) * 20
            ee_pos = positions[-1]
            for sign in [-1, 1]:
                g = (ee_pos[0] + sign * grip_open, ee_pos[1] + 25, ee_pos[2])
                gs = project_3d(*g, self.view_h, self.view_v, cx, cy)
                pygame.draw.line(self.screen, (180,190,200), (ee[0],ee[1]), (gs[0],gs[1]), 2)
                pygame.draw.circle(self.screen, JOINT_COLORS[5], (int(gs[0]),int(gs[1])), 3)

        # Joints
        for i, p in enumerate(projected):
            r = max(3, int(6 * p[3]))
            color = JOINT_COLORS[i] if i < len(JOINT_COLORS) else (255,255,255)
            pygame.draw.circle(self.screen, color, (int(p[0]),int(p[1])), r)
            pygame.draw.circle(self.screen, (255,255,255), (int(p[0]),int(p[1])), max(1, r//2))

        # End effector glow
        if projected:
            ee = projected[-1]
            pulse = int(3 * (1 + math.sin(time.time() * 4)))
            glow = pygame.Surface(((8+pulse)*4, (8+pulse)*4), pygame.SRCALPHA)
            pygame.draw.circle(glow, (*EE_COLOR, 40), ((8+pulse)*2, (8+pulse)*2), (8+pulse)*2)
            self.screen.blit(glow, (int(ee[0])-(8+pulse)*2, int(ee[1])-(8+pulse)*2))
            pygame.draw.circle(self.screen, EE_COLOR, (int(ee[0]),int(ee[1])), 6)

        # IK target
        if self.ik_dragging and self.ik_target is not None:
            tx, ty, _, _ = project_3d(*self.ik_target, self.view_h, self.view_v, cx, cy)
            pygame.draw.circle(self.screen, TARGET_COLOR, (int(tx),int(ty)), 8, 2)
            pygame.draw.line(self.screen, TARGET_COLOR, (int(tx)-10,int(ty)), (int(tx)+10,int(ty)), 1)
            pygame.draw.line(self.screen, TARGET_COLOR, (int(tx),int(ty)-10), (int(tx),int(ty)+10), 1)

        # IK drag hint
        if projected:
            ee = projected[-1]
            mp = pygame.mouse.get_pos()
            if math.hypot(mp[0]-ee[0], mp[1]-ee[1]) < 15:
                self.screen.blit(self.font_xs.render("drag to move (IK)", True, TARGET_COLOR),
                                 (int(ee[0])+12, int(ee[1])-8))

        # Orbit label
        self.screen.blit(self.font_xs.render(f"orbit: ({self.view_h:.0f}\u00b0, {self.view_v:.0f}\u00b0)",
                                              True, TEXT_DIM), (vp.x+8, vp.bottom-18))

    def _draw_waypoint_ghosts(self, cx, cy):
        """Draw faint arm outlines at each saved waypoint position."""
        ghost_color = (60, 120, 200)
        for i, name in enumerate(self.wp_manager.program):
            wp = self.wp_manager.waypoints.get(name)
            if not wp: continue

            # Compute FK for this waypoint
            from arm_controller import forward_kinematics as fk
            positions, _ = fk(wp.angles)
            projected = []
            for pos in positions:
                sx, sy, sz, sf = project_3d(pos[0], pos[1], pos[2], self.view_h, self.view_v, cx, cy)
                projected.append((sx, sy, sz, sf))

            # Draw ghost links
            alpha = 40 if i != self.selected_wp_idx else 80
            for j in range(len(projected) - 1):
                p1, p2 = projected[j], projected[j+1]
                pygame.draw.line(self.screen, (*ghost_color, alpha) if alpha < 255 else ghost_color,
                                 (p1[0], p1[1]), (p2[0], p2[1]), 1)

            # Ghost end effector dot with label
            if projected:
                ee = projected[-1]
                r = 4 if i == self.selected_wp_idx else 3
                pygame.draw.circle(self.screen, ghost_color, (int(ee[0]), int(ee[1])), r)
                label = self.font_xs.render(f"{i+1}", True, ghost_color)
                self.screen.blit(label, (int(ee[0]) + 6, int(ee[1]) - 10))


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    app = RobotArmApp()
    app.run()
