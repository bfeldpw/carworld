const std = @import("std");
const bfe = @import("bfe");
const main = @import("main.zig");
const c = bfe.gfx.c.c;
const gfx_cam = bfe.gfx.cam;
const ipt = bfe.input;

//-----------------------------------------------------------------------------//
//   Error Sets / Enums
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Init / DeInit
//-----------------------------------------------------------------------------//

pub fn init() !void {
    try ipt.addPlugin(&process);
}

//-----------------------------------------------------------------------------//
//   Getter/Setter
//-----------------------------------------------------------------------------//

//-----------------------------------------------------------------------------//
//   Internal
//-----------------------------------------------------------------------------//

const log_input = std.log.scoped(.input);

//-----------------------------------------------------------------------------//
//   Processing
//-----------------------------------------------------------------------------//

fn process() void {
    if (ipt.getKeyState(.key_ctrl_left)) {
        if (ipt.getKeyPressEvent(.key_q)) ipt.closeWindow();
        if (ipt.getKeyPressEvent(.key_r)) {
            gfx_cam.reset();
            gfx_cam.zoomToWidth(150.0);
        }
        if (ipt.getKeyPressEvent(.key_p)) main.togglePause();
    }
    // const shift = ipt.getKeyState(.key_shift_left);
    if (ipt.getKeyState(.key_left)) gfx_cam.moveVelRel(-0.5, 0);
    if (ipt.getKeyState(.key_right)) gfx_cam.moveVelRel(0.5, 0);
    if (ipt.getKeyState(.key_up)) gfx_cam.moveVelRel(0, 0.5);
    if (ipt.getKeyState(.key_down)) gfx_cam.moveVelRel(0, -0.5);
    if (ipt.getKeyState(.key_page_up)) gfx_cam.zoomBy(-0.05);
    if (ipt.getKeyState(.key_page_down)) gfx_cam.zoomBy(0.05);

}
