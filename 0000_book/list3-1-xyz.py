import numpy as np
from wrs import modeling as mgm
import wrs.visualization.panda.world as wd

if __name__ == '__main__':
    base = wd.World(cam_pos=np.array([1, 1, 1]), lookat_pos=np.zeros(3))
    frame_model = mgm.gen_frame(ax_length=.2)
    frame_model.attach_to(base)
    base.run()