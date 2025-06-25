const std = @import("std");
const bfe = @import("bfe");
const input = @import("input_plugin.zig");

pub fn main() !void {

    // Init
    bfe.gfx.core.setFpsTarget(60.0);
    try bfe.gfx.core.init();
    try bfe.input.init(bfe.gfx.core.getWindow());
    try input.init();
    defer bfe.input.deinit();
    defer bfe.gfx.core.deinit();

    // Run
    while (bfe.gfx.core.isWindowOpen()) {
        bfe.input.process();
        try bfe.gfx.core.finishFrame();
    }
}
