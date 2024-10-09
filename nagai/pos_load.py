from wrs import wd, rm, mcm, mgm
import nagai as khi
import yaml
import os

if __name__ == '__main__':    
    class Data(object):
        def __init__(self, mot_data):
            self.counter = 0
            self.mot_data = mot_data

    base = wd.World(cam_pos=[.5,.5,.5], lookat_pos=[0, 0, 0])
    mgm.gen_frame(ax_length=.15).attach_to(base)

    # load objects
    with open(os.path.join(khi.__path__[0], "object.yaml")) as f:
        objects = yaml.safe_load(f)

    # workbench
    workbench_file = os.path.join(khi.__path__[0], "meshes", objects['object1']['name'] + ".stl")
    workbench_cm = mcm.CollisionModel(initor=workbench_file,rgb=rm.const.orange_red)
    workbench_cm.pos = rm.np.array(objects['object1']['pos']) #+ rm.np.array([1,1,1])
    workbench_cm.rotmat = rm.np.dot(rm.np.array(objects['object1']['rotmat']), rm.rotmat_from_euler(ai=rm.np.pi/2,aj=0,ak=0,order='rxyz'))
    workbench_cm.attach_to(base)

    rot_o2h = workbench_cm.rotmat
    pos_o2h = workbench_cm.pos

    # bracketR1
    bracketR1_file = os.path.join(khi.__path__[0], "meshes", objects['object2']['name'] + ".stl")
    bracketR1_cm = mcm.CollisionModel(initor=bracketR1_file,rgb=rm.const.gray)
    bracketR1_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['pos'])) + pos_o2h
    bracketR1_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['rotmat']))
    bracketR1_cm.attach_to(base)
    
    # capacitor
    capacitor_file = os.path.join(khi.__path__[0], "meshes", objects['object3']['name'] + ".stl")
    capacitor_cm = mcm.CollisionModel(initor=capacitor_file,rgb=rm.const.blue)
    capacitor_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['pos'])) + pos_o2h
    capacitor_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['rotmat']))
    capacitor_cm.attach_to(base)

    # relay_205B
    relay_205B_file = os.path.join(khi.__path__[0], "meshes", objects['object4']['name'] + ".stl")
    relay_205B_cm = mcm.CollisionModel(initor=relay_205B_file,rgb=rm.const.black)
    relay_205B_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['pos'])) + pos_o2h
    relay_205B_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['rotmat']))
    relay_205B_cm.attach_to(base)

    # belt
    belt_file = os.path.join(khi.__path__[0], "meshes", objects['object5']['name'] + ".stl")
    belt_cm = mcm.CollisionModel(initor=belt_file,rgb=rm.const.deep_sky_blue)
    belt_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['pos'])) + pos_o2h
    belt_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['rotmat']))
    belt_cm.attach_to(base)

    # terminal_block
    terminal_block_file = os.path.join(khi.__path__[0], "meshes", objects['object6']['name'] + ".stl")
    terminal_block_cm = mcm.CollisionModel(initor=terminal_block_file,rgb=rm.const.yellow)
    terminal_block_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['pos'])) + pos_o2h
    terminal_block_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['rotmat']))
    terminal_block_cm.attach_to(base)

    #numpy print options
    rm.np.set_printoptions(precision=3,suppress=True)

    #print rotmats and pos
    print("workbench rotmat is:\t pos is:")
    print(workbench_cm.rotmat, workbench_cm.pos)
    print("bracketR1 rotmat is:\t pos is:")
    print(bracketR1_cm.rotmat, bracketR1_cm.pos)
    print("capacitor rotmat is:\t pos is:")
    print(capacitor_cm.rotmat, capacitor_cm.pos)
    print("relay____ rotmat is:\t pos is:")
    print(relay_205B_cm.rotmat, relay_205B_cm.pos)
    print("belt_____ rotmat is:\t pos is:")
    print(belt_cm.rotmat, belt_cm.pos)
    print("terminal_ rotmat is:\t pos is:")
    print(terminal_block_cm.rotmat, terminal_block_cm.pos)

    # workbench collision check
    print("workbench collision check")
    print("bracket collided?:", workbench_cm.is_mcdwith(bracketR1_cm))
    print("capacitor collided?:", workbench_cm.is_mcdwith(capacitor_cm))
    print("relay collided?:", workbench_cm.is_mcdwith(relay_205B_cm))
    print("belt collided?:", workbench_cm.is_mcdwith(belt_cm))
    print("terminal collided?:", workbench_cm.is_mcdwith(terminal_block_cm))

    # bracketR1 collision check
    print("\nbracketR1 collision check")
    print("workbench collided?:", bracketR1_cm.is_mcdwith(workbench_cm))
    print("capacitor collided?:", bracketR1_cm.is_mcdwith(capacitor_cm))
    print("relay collided?:", bracketR1_cm.is_mcdwith(relay_205B_cm))
    print("belt collided?:", bracketR1_cm.is_mcdwith(belt_cm))
    print("terminal collided?:", bracketR1_cm.is_mcdwith(terminal_block_cm))

    base.run()