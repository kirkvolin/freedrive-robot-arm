"""
Robot Arm Simulator
====================
3D visualization with joint sliders and IK end-effector dragging.
"""

import sys
import math
import time
import pygame
import numpy as np
from arm_controller import ArmController, TrajectoryRecorder, forward_kinematics

BG             = (13, 17, 23)
PANEL_BG       = (22, 27, 34)
BORDER         = (48, 54, 61)
TEXT           = (201, 209, 217)
TEXT_DIM       = (106, 115, 125)
GRID_COLOR     = (26, 35, 50)

JOINT_COLORS = [
    (231, 76, 60),    # J1 red
    (230, 126, 34),   # J2 orange
    (241, 196, 15),   # J3 yellow
    (46, 204, 113),   # J4 green
    (52, 152, 219),   # J5 blue
    (155, 89, 182),   # J6 purple
]

LINK_COLOR     = (120, 144, 168)
AXIS_X         = (231, 76, 60)
AXIS_Y         = (46, 204, 113)
AXIS_Z         = (52, 152, 219)
EE_COLOR       = (255, 100, 100)
TARGET_COLOR   = (255, 255, 100)
GHOST_COLOR    = (100, 100, 100)

STATUS_READY   = (46, 204, 113)
STATUS_REC     = (231, 76, 60)
STATUS_PLAY    = (52, 152, 219)
STATUS_FREE    = (241, 196, 15)

# ============================================================
# 3D PROJECTION
# ============================================================

def project_3d(x, y, z, view_h, view_v, cx, cy, scale=1.1):
    """Project 3D point to 2D screen coordinates."""
    rad_h = math.radians(view_h)
    rad_v = math.radians(view_v)

    # Horizontal orbit (around Y)
    x1 = x * math.cos(rad_h) - z * math.sin(rad_h)
    z1 = x * math.sin(rad_h) + z * math.cos(rad_h)
    y1 = y

    # Vertical tilt (around X)
    y2 = y1 * math.cos(rad_v) - z1 * math.sin(rad_v)
    z2 = y1 * math.sin(rad_v) + z1 * math.cos(rad_v)
    x2 = x1

    perspective = 800
    factor = perspective / (perspective + z2)

    sx = cx + x2 * scale * factor
    sy = cy - y2 * scale * factor
    return sx, sy, z2, factor


def screen_to_world_ray(sx, sy, view_h, view_v, cx, cy, scale=1.1):
    """Approximate inverse projection for IK dragging."""
    rad_h = math.radians(-view_h)
    rad_v = math.radians(-view_v)

    # Undo screen transform (approximate, assumes z=0 plane initially)
    nx = (sx - cx) / scale
    ny = -(sy - cy) / scale

    # Undo vertical tilt
    y1 = ny * math.cos(rad_v)
    z1 = ny * math.sin(rad_v)
    x1 = nx

    # Undo horizontal orbit
    x = x1 * math.cos(rad_h) - z1 * math.sin(rad_h)
    z = x1 * math.sin(rad_h) + z1 * math.cos(rad_h)
    y = y1

    return np.array([x, y, z])


# ============================================================
# UI COMPONENTS
# ============================================================

class Slider:
    """Horizontal slider for joint control."""

    def __init__(self, x, y, w, h, min_val, max_val, value, color, label):
        self.rect = pygame.Rect(x, y, w, h)
        self.min_val = min_val
        self.max_val = max_val
        self.value = value
        self.color = color
        self.label = label
        self.dragging = False
        self.track_rect = pygame.Rect(x, y + h // 2 - 2, w, 4)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            # Wider hit area for the track
            hit_rect = pygame.Rect(self.rect.x, self.rect.y + 8, self.rect.w, self.rect.h - 8)
            if hit_rect.collidepoint(event.pos):
                self.dragging = True
                self._update_value(event.pos[0])
                return True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self._update_value(event.pos[0])
            return True
        return False

    def _update_value(self, mouse_x):
        t = (mouse_x - self.rect.x) / self.rect.w
        t = max(0, min(1, t))
        self.value = self.min_val + t * (self.max_val - self.min_val)

    def draw(self, surface, font_sm, font_xs):
        # Label and value
        label_surf = font_sm.render(self.label, True, self.color)
        val_surf = font_sm.render(f"{self.value:.1f}°", True, self.color)
        surface.blit(label_surf, (self.rect.x, self.rect.y))
        surface.blit(val_surf, (self.rect.right - val_surf.get_width(), self.rect.y))

        # Track
        track_y = self.rect.y + 22
        pygame.draw.rect(surface, BORDER, (self.rect.x, track_y, self.rect.w, 4), border_radius=2)

        # Fill
        t = (self.value - self.min_val) / (self.max_val - self.min_val)
        fill_w = int(self.rect.w * t)
        if fill_w > 0:
            fill_color = (*self.color[:3], 180) if len(self.color) == 3 else self.color
            pygame.draw.rect(surface, self.color, (self.rect.x, track_y, fill_w, 4), border_radius=2)

        # Thumb
        thumb_x = self.rect.x + fill_w
        pygame.draw.circle(surface, self.color, (thumb_x, track_y + 2), 6)
        pygame.draw.circle(surface, (255, 255, 255), (thumb_x, track_y + 2), 3)


class Button:
    """Simple button."""

    def __init__(self, x, y, w, h, label, color, font):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.color = color
        self.font = font
        self.hovered = False

    def handle_event(self, event):
        if event.type == pygame.MOUSEMOTION:
            self.hovered = self.rect.collidepoint(event.pos)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            if self.rect.collidepoint(event.pos):
                return True
        return False

    def draw(self, surface, active=False):
        border_color = self.color if (active or self.hovered) else BORDER
        bg = (*self.color, 30) if active else PANEL_BG
        pygame.draw.rect(surface, PANEL_BG, self.rect, border_radius=4)
        pygame.draw.rect(surface, border_color, self.rect, 1, border_radius=4)

        text_color = self.color if active else TEXT_DIM
        label_surf = self.font.render(self.label, True, text_color)
        lx = self.rect.centerx - label_surf.get_width() // 2
        ly = self.rect.centery - label_surf.get_height() // 2
        surface.blit(label_surf, (lx, ly))


# ============================================================
# MAIN APPLICATION
# ============================================================

class RobotArmApp:
    def __init__(self):
        pygame.init()

        # Window
        self.width, self.height = 1100, 700
        self.screen = pygame.display.set_mode((self.width, self.height), pygame.RESIZABLE)
        pygame.display.set_caption("6-DOF Robot Arm Simulator")

        # Fonts
        self.font_lg = pygame.font.SysFont("monospace", 16, bold=True)
        self.font_md = pygame.font.SysFont("monospace", 13)
        self.font_sm = pygame.font.SysFont("monospace", 11)
        self.font_xs = pygame.font.SysFont("monospace", 10)

        # Robot
        self.controller = ArmController()
        self.recorder = TrajectoryRecorder()
        self.controller.home_all()

        # View
        self.view_h = 35
        self.view_v = 25
        self.orbiting = False
        self.orbit_start = (0, 0)
        self.orbit_start_angles = (0, 0)

        # IK drag
        self.ik_dragging = False
        self.ik_target = None

        # Playback
        self.playing = False
        self.play_index = 0
        self.play_timer = 0

        # Viewport area
        self.viewport_rect = pygame.Rect(0, 0, 680, 560)
        self.panel_x = 690

        # Build UI
        self._build_ui()

        # Clock
        self.clock = pygame.time.Clock()
        self.running = True

    def _build_ui(self):
        """Create sliders and buttons."""
        sx = self.panel_x
        sy = 80

        self.sliders = []
        for i, servo in enumerate(self.controller.servos):
            slider = Slider(sx, sy + i * 52, 370, 40,
                            servo.min_angle, servo.max_angle,
                            servo.get_angle(), JOINT_COLORS[i], servo.label)
            self.sliders.append(slider)

        btn_y = sy + 6 * 52 + 10
        btn_w = 86
        btn_h = 30
        gap = 6

        self.btn_home = Button(sx, btn_y, btn_w, btn_h, "HOME", (52, 152, 219), self.font_sm)
        self.btn_free = Button(sx + (btn_w + gap), btn_y, btn_w, btn_h, "FREEDRV", (241, 196, 15), self.font_sm)
        self.btn_rec  = Button(sx + 2 * (btn_w + gap), btn_y, btn_w, btn_h, "REC", (231, 76, 60), self.font_sm)
        self.btn_play = Button(sx + 3 * (btn_w + gap), btn_y, btn_w, btn_h, "PLAY", (46, 204, 113), self.font_sm)

        self.buttons = [self.btn_home, self.btn_free, self.btn_rec, self.btn_play]

        # Presets
        preset_y = btn_y + 45
        preset_w = 110
        self.presets = [
            ("Home",         [0, 0, 0, 0, 0, 50]),
            ("Reach Fwd",    [0, 45, -30, -15, 0, 50]),
            ("Reach Left",   [90, 30, -20, 0, 0, 50]),
            ("Pick Ready",   [0, 60, -90, 30, 0, 100]),
            ("Compact",      [0, -60, 90, -30, 0, 0]),
            ("Wave",         [45, 20, -45, 60, 90, 80]),
        ]
        self.preset_buttons = []
        for i, (name, _) in enumerate(self.presets):
            col = i % 3
            row = i // 3
            bx = sx + col * (preset_w + gap)
            by = preset_y + row * (btn_h + gap)
            self.preset_buttons.append(Button(bx, by, preset_w, btn_h, name, TEXT_DIM, self.font_xs))

    def run(self):
        while self.running:
            dt = self.clock.tick(60) / 1000.0
            self._handle_events()
            self._update(dt)
            self._draw()
        pygame.quit()

    def _handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                return

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_h:
                    self.controller.home_all()
                    self._sync_sliders()
                elif event.key == pygame.K_f:
                    self._toggle_freedrive()
                elif event.key == pygame.K_r:
                    self._toggle_recording()
                elif event.key == pygame.K_p:
                    self._toggle_playback()

            # Button events
            if self.btn_home.handle_event(event):
                self.controller.home_all()
                self._sync_sliders()
            if self.btn_free.handle_event(event):
                self._toggle_freedrive()
            if self.btn_rec.handle_event(event):
                self._toggle_recording()
            if self.btn_play.handle_event(event):
                self._toggle_playback()

            for i, btn in enumerate(self.preset_buttons):
                if btn.handle_event(event):
                    _, preset_angles = self.presets[i]
                    for j, a in enumerate(preset_angles):
                        self.controller.set_joint_angle(j, a)
                    self._sync_sliders()

            # Slider events
            if not self.playing:
                for i, slider in enumerate(self.sliders):
                    if slider.handle_event(event):
                        self.controller.set_joint_angle(i, slider.value)

            # Viewport interaction
            if event.type == pygame.MOUSEBUTTONDOWN:
                if self.viewport_rect.collidepoint(event.pos):
                    if event.button == 1:
                        # Check if clicking near end effector for IK drag
                        if self._near_end_effector(event.pos):
                            self.ik_dragging = True
                            self.ik_target = self.controller.get_end_effector_pos().copy()
                        else:
                            self.orbiting = True
                            self.orbit_start = event.pos
                            self.orbit_start_angles = (self.view_h, self.view_v)
                    elif event.button == 3:
                        # Right-click always starts IK drag
                        self.ik_dragging = True
                        self.ik_target = self.controller.get_end_effector_pos().copy()

            elif event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    self.orbiting = False
                    self.ik_dragging = False
                elif event.button == 3:
                    self.ik_dragging = False

            elif event.type == pygame.MOUSEMOTION:
                if self.orbiting:
                    dx = event.pos[0] - self.orbit_start[0]
                    dy = event.pos[1] - self.orbit_start[1]
                    self.view_h = self.orbit_start_angles[0] + dx * 0.5
                    self.view_v = max(-60, min(80, self.orbit_start_angles[1] + dy * 0.3))

                elif self.ik_dragging and not self.playing:
                    # Map mouse movement to world delta
                    vp_cx = self.viewport_rect.centerx
                    vp_cy = self.viewport_rect.y + self.viewport_rect.h * 0.75
                    target_display = screen_to_world_ray(
                        event.pos[0], event.pos[1],
                        self.view_h, self.view_v,
                        vp_cx, vp_cy
                    )
                    # display Y is FK Z and vice versa — swap back to FK space
                    target_fk = np.array([target_display[0], target_display[2], target_display[1]])
                    self.ik_target = target_fk
                    self.controller.move_to_position(target_fk)
                    self._sync_sliders()

    def _near_end_effector(self, mouse_pos, threshold=15):
        """Check if mouse is near the projected end effector."""
        ee = self.controller.get_end_effector_pos()
        vp_cx = self.viewport_rect.centerx
        vp_cy = self.viewport_rect.y + self.viewport_rect.h * 0.75
        sx, sy, _, _ = project_3d(ee[0], ee[2], ee[1], self.view_h, self.view_v, vp_cx, vp_cy)
        dist = math.hypot(mouse_pos[0] - sx, mouse_pos[1] - sy)
        return dist < threshold

    def _toggle_freedrive(self):
        self.controller.freedrive = not self.controller.freedrive
        self.controller.set_freedrive(self.controller.freedrive)

    def _toggle_recording(self):
        if not self.recorder.recording:
            self.recorder.start_recording()
        else:
            self.recorder.stop_recording()

    def _toggle_playback(self):
        if self.playing:
            self.playing = False
        elif self.recorder.frame_count > 0:
            self.playing = True
            self.play_index = 0
            self.play_timer = 0

    def _sync_sliders(self):
        """Update slider values from controller state."""
        angles = self.controller.get_joint_angles()
        for i, slider in enumerate(self.sliders):
            slider.value = angles[i]

    def _update(self, dt):
        # Recording
        if self.recorder.recording:
            self.recorder.record_frame(self.controller.get_joint_angles(), time.time())

        # Playback
        if self.playing:
            self.play_timer += dt
            if self.play_timer >= self.recorder.record_interval:
                self.play_timer = 0
                frame = self.recorder.get_frame(self.play_index)
                if frame:
                    for i, angle in enumerate(frame):
                        self.controller.set_joint_angle(i, angle)
                    self._sync_sliders()
                    self.play_index += 1
                else:
                    self.playing = False
                    self.play_index = 0

    def _draw(self):
        self.screen.fill(BG)

        # Draw viewport
        self._draw_viewport()

        # Draw panel background
        panel_rect = pygame.Rect(self.panel_x - 10, 0, self.width - self.panel_x + 10, self.height)
        pygame.draw.rect(self.screen, PANEL_BG, panel_rect)
        pygame.draw.line(self.screen, BORDER, (self.panel_x - 10, 0), (self.panel_x - 10, self.height))

        # Header
        status_color = STATUS_FREE if self.controller.freedrive else (
            STATUS_REC if self.recorder.recording else (
                STATUS_PLAY if self.playing else STATUS_READY
            )
        )
        status_text = "FREEDRIVE" if self.controller.freedrive else (
            "● RECORDING" if self.recorder.recording else (
                "▶ PLAYING" if self.playing else "READY"
            )
        )

        pygame.draw.circle(self.screen, status_color, (self.panel_x + 8, 22), 4)
        title = self.font_lg.render("6-DOF ARM CONTROL", True, TEXT)
        self.screen.blit(title, (self.panel_x + 20, 13))
        stat = self.font_xs.render(status_text, True, status_color)
        self.screen.blit(stat, (self.panel_x + 20, 36))

        # End effector readout
        ee = self.controller.get_end_effector_pos()
        ee_text = f"TCP: ({ee[0]:.1f}, {ee[1]:.1f}, {ee[2]:.1f}) mm"
        ee_surf = self.font_sm.render(ee_text, True, TEXT_DIM)
        self.screen.blit(ee_surf, (self.panel_x + 20, 54))

        # Sliders
        for slider in self.sliders:
            slider.draw(self.screen, self.font_sm, self.font_xs)

        # Buttons
        self.btn_home.draw(self.screen)
        self.btn_free.draw(self.screen, active=self.controller.freedrive)
        self.btn_rec.draw(self.screen, active=self.recorder.recording)
        self.btn_play.draw(self.screen, active=self.playing)

        # Trajectory info
        if self.recorder.frame_count > 0:
            info = f"Trajectory: {self.recorder.frame_count} frames ({self.recorder.duration:.1f}s)"
            if self.playing:
                info += f"  [{self.play_index}/{self.recorder.frame_count}]"
            info_surf = self.font_xs.render(info, True, TEXT_DIM)
            btn_y = self.sliders[-1].rect.bottom + 60
            self.screen.blit(info_surf, (self.panel_x, btn_y - 5))

        # Preset buttons
        presets_label = self.font_xs.render("PRESETS", True, TEXT_DIM)
        if self.preset_buttons:
            self.screen.blit(presets_label, (self.panel_x, self.preset_buttons[0].rect.y - 18))
        for btn in self.preset_buttons:
            btn.draw(self.screen)

        # Help text at bottom
        help_y = self.height - 60
        help_lines = [
            "Keys: [H]ome  [F]reedrive  [R]ecord  [P]lay  [Esc]Quit",
            "Mouse: Left-drag=orbit  Right-drag/click end-effector=IK drag",
        ]
        for i, line in enumerate(help_lines):
            surf = self.font_xs.render(line, True, TEXT_DIM)
            self.screen.blit(surf, (self.panel_x, help_y + i * 16))

        pygame.display.flip()

    def _draw_viewport(self):
        """Draw the 3D arm visualization."""
        vp = self.viewport_rect
        pygame.draw.rect(self.screen, (8, 12, 18), vp)
        pygame.draw.rect(self.screen, BORDER, vp, 1)

        cx = vp.centerx
        cy = vp.y + vp.h * 0.75

        # Grid
        grid_size = 200
        grid_step = 50
        for i in range(-grid_size, grid_size + 1, grid_step):
            ax, ay, _, _ = project_3d(i, 0, -grid_size, self.view_h, self.view_v, cx, cy)
            bx, by, _, _ = project_3d(i, 0, grid_size, self.view_h, self.view_v, cx, cy)
            if vp.clipline(ax, ay, bx, by):
                pygame.draw.line(self.screen, GRID_COLOR, (ax, ay), (bx, by), 1)
            ax, ay, _, _ = project_3d(-grid_size, 0, i, self.view_h, self.view_v, cx, cy)
            bx, by, _, _ = project_3d(grid_size, 0, i, self.view_h, self.view_v, cx, cy)
            if vp.clipline(ax, ay, bx, by):
                pygame.draw.line(self.screen, GRID_COLOR, (ax, ay), (bx, by), 1)

        # Axes
        ox, oy, _, _ = project_3d(0, 0, 0, self.view_h, self.view_v, cx, cy)
        for end, color, label in [
            ((60, 0, 0), AXIS_X, "X"),
            ((0, 60, 0), AXIS_Y, "Y"),
            ((0, 0, 60), AXIS_Z, "Z"),
        ]:
            ex, ey, _, _ = project_3d(*end, self.view_h, self.view_v, cx, cy)
            pygame.draw.line(self.screen, color, (ox, oy), (ex, ey), 1)
            label_surf = self.font_xs.render(label, True, color)
            self.screen.blit(label_surf, (ex + 3, ey - 5))

        # Arm links and joints
        positions = self.controller.get_fk_positions()
        projected = []
        for pos in positions:
            # FK Z is world-up; swap Y↔Z so the arm stands vertically on screen
            sx, sy, sz, sf = project_3d(pos[0], pos[2], pos[1], self.view_h, self.view_v, cx, cy)
            projected.append((sx, sy, sz, sf))

        # Draw links
        for i in range(len(projected) - 1):
            p1 = projected[i]
            p2 = projected[i + 1]
            thickness = max(2, int(5 * p1[3]))
            pygame.draw.line(self.screen, LINK_COLOR, (p1[0], p1[1]), (p2[0], p2[1]), thickness)

        # Draw gripper
        if len(projected) >= 5:
            ee = projected[-1]
            gripper_angle = self.controller.get_joint_angles()[5]
            grip_open = (gripper_angle / 100) * 20

            # Gripper fingers (simplified) — extend downward along FK -Z from EE tip
            ee_pos = positions[-1]
            gl = (ee_pos[0] - grip_open, ee_pos[1], ee_pos[2] - 25)
            gr = (ee_pos[0] + grip_open, ee_pos[1], ee_pos[2] - 25)
            gl_s = project_3d(gl[0], gl[2], gl[1], self.view_h, self.view_v, cx, cy)
            gr_s = project_3d(gr[0], gr[2], gr[1], self.view_h, self.view_v, cx, cy)

            pygame.draw.line(self.screen, (180, 190, 200), (ee[0], ee[1]), (gl_s[0], gl_s[1]), 2)
            pygame.draw.line(self.screen, (180, 190, 200), (ee[0], ee[1]), (gr_s[0], gr_s[1]), 2)
            pygame.draw.circle(self.screen, JOINT_COLORS[5], (int(gl_s[0]), int(gl_s[1])), 3)
            pygame.draw.circle(self.screen, JOINT_COLORS[5], (int(gr_s[0]), int(gr_s[1])), 3)

        # Draw joints
        for i, p in enumerate(projected):
            r = max(3, int(6 * p[3]))
            color = JOINT_COLORS[i] if i < len(JOINT_COLORS) else (255, 255, 255)
            pygame.draw.circle(self.screen, color, (int(p[0]), int(p[1])), r)
            pygame.draw.circle(self.screen, (255, 255, 255), (int(p[0]), int(p[1])), max(1, r // 2))

        # End effector highlight (larger, pulsing)
        if len(projected) > 0:
            ee = projected[-1]
            pulse = int(3 * (1 + math.sin(time.time() * 4)))
            r = 8 + pulse
            # Outer glow
            glow_surf = pygame.Surface((r * 4, r * 4), pygame.SRCALPHA)
            pygame.draw.circle(glow_surf, (*EE_COLOR, 40), (r * 2, r * 2), r * 2)
            self.screen.blit(glow_surf, (int(ee[0]) - r * 2, int(ee[1]) - r * 2))
            # Inner dot
            pygame.draw.circle(self.screen, EE_COLOR, (int(ee[0]), int(ee[1])), 6)

        # IK target indicator
        if self.ik_dragging and self.ik_target is not None:
            tx, ty, _, _ = project_3d(
                self.ik_target[0], self.ik_target[2], self.ik_target[1],
                self.view_h, self.view_v, cx, cy
            )
            pygame.draw.circle(self.screen, TARGET_COLOR, (int(tx), int(ty)), 8, 2)
            pygame.draw.line(self.screen, TARGET_COLOR, (int(tx) - 10, int(ty)), (int(tx) + 10, int(ty)), 1)
            pygame.draw.line(self.screen, TARGET_COLOR, (int(tx), int(ty) - 10), (int(tx), int(ty) + 10), 1)

        # IK drag hint near end effector
        if len(projected) > 0:
            ee = projected[-1]
            mouse_pos = pygame.mouse.get_pos()
            if self._near_end_effector_proj(mouse_pos, ee):
                hint = self.font_xs.render("drag to move (IK)", True, TARGET_COLOR)
                self.screen.blit(hint, (int(ee[0]) + 12, int(ee[1]) - 8))

        # Viewport label
        label = self.font_xs.render(f"orbit: ({self.view_h:.0f}°, {self.view_v:.0f}°)", True, TEXT_DIM)
        self.screen.blit(label, (vp.x + 8, vp.bottom - 18))

    def _near_end_effector_proj(self, mouse_pos, ee_proj, threshold=15):
        dist = math.hypot(mouse_pos[0] - ee_proj[0], mouse_pos[1] - ee_proj[1])
        return dist < threshold


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    app = RobotArmApp()
    app.run()
