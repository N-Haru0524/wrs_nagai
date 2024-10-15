from wrs import wd, rm, mcm, mgm
import nagai as khi
import yaml
import os
from nagai import worklist

worklist = worklist.WORK_LIST()

worklist.bracketR1_cm.alpha = .3
worklist.capacitor_cm.alpha = .3
worklist.relay_205B_cm.alpha = .3
worklist.belt_cm.alpha = .3
worklist.terminal_block_cm.alpha = .3
# worklist.workbench_cm.alpha = .3

worklist.workbench_cm.show_local_frame()

base = wd.World(cam_pos=[.5,.5,.5], lookat_pos=[0, 0, 0])
mgm.gen_frame(ax_length=.15).attach_to(base)

worklist.attach_to(base)
base.run()