from wrs import wd, rm, mcm, mgm
import nagai as khi
import yaml
import os

class WORK_LIST():
    def __init__(self,
                 pos = rm.np.array([0,0,0]),
                 rotmat = rm.rotmat_from_euler(ai=rm.np.pi/2,aj=0,ak=0,order='rxyz'),
                 yamlpath = khi.__path__[0],
                 meshpath=khi.__path__[0]):
        with open(os.path.join(yamlpath, "object.yaml")) as f:
            objects = yaml.safe_load(f)
        # workbench
        workbench_file = os.path.join(meshpath, "meshes", objects['object1']['name'] + ".stl")
        self.workbench_cm = mcm.CollisionModel(initor=workbench_file,rgb=rm.const.orange_red)
        self.workbench_cm.pos = rm.np.array(objects['object1']['pos']) + pos
        self.workbench_cm.rotmat = rm.np.dot(rm.np.array(objects['object1']['rotmat']), rotmat)

        rot_o2h = self.workbench_cm.rotmat
        pos_o2h = self.workbench_cm.pos

        # bracketR1
        bracketR1_file = os.path.join(meshpath, "meshes", objects['object2']['name'] + ".stl")
        self.bracketR1_cm = mcm.CollisionModel(initor=bracketR1_file,rgb=rm.const.gray)
        self.bracketR1_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['pos'])) + pos_o2h
        self.bracketR1_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['rotmat']))

        # capacitor
        capacitor_file = os.path.join(meshpath, "meshes", objects['object3']['name'] + ".stl")
        self.capacitor_cm = mcm.CollisionModel(initor=capacitor_file,rgb=rm.const.blue)
        self.capacitor_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['pos'])) + pos_o2h
        self.capacitor_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['rotmat']))

        # relay_205B
        relay_205B_file = os.path.join(meshpath, "meshes", objects['object4']['name'] + ".stl")
        self.relay_205B_cm = mcm.CollisionModel(initor=relay_205B_file,rgb=rm.const.black)
        self.relay_205B_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['pos'])) + pos_o2h
        self.relay_205B_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['rotmat']))

        # belt
        belt_file = os.path.join(meshpath, "meshes", objects['object5']['name'] + ".stl")
        self.belt_cm = mcm.CollisionModel(initor=belt_file,rgb=rm.const.deep_sky_blue)
        self.belt_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['pos'])) + pos_o2h
        self.belt_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['rotmat']))

        # terminal_block
        terminal_block_file = os.path.join(meshpath, "meshes", objects['object6']['name'] + ".stl")
        self.terminal_block_cm = mcm.CollisionModel(initor=terminal_block_file,rgb=rm.const.yellow)
        self.terminal_block_cm.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['pos'])) + pos_o2h
        self.terminal_block_cm.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['rotmat']))

    def attach_to(self, base):
        self.workbench_cm.attach_to(base)
        self.bracketR1_cm.attach_to(base)
        self.capacitor_cm.attach_to(base)
        self.relay_205B_cm.attach_to(base)
        self.belt_cm.attach_to(base)
        self.terminal_block_cm.attach_to(base)

if __name__ == '__main__':
    class Data(object):
        def __init__(self, mot_data):
            self.counter = 0
            self.mot_data = mot_data

    base = wd.World(cam_pos=[.5,.5,.5], lookat_pos=[0, 0, 0])
    mgm.gen_frame(ax_length=.15).attach_to(base)

    worklist = WORK_LIST(rotmat=rm.np.eye(3))
    worklist.attach_to(base)

    #numpy print options
    rm.np.set_printoptions(precision=3,suppress=True)
    
    #print rotmats and pos
    print("workbench rotmat is:\t pos is:")
    print(worklist.workbench_cm.rotmat, worklist.workbench_cm.pos)
    print("bracketR1 rotmat is:\t pos is:")
    print(worklist.bracketR1_cm.rotmat, worklist.bracketR1_cm.pos)
    print("capacitor rotmat is:\t pos is:")
    print(worklist.capacitor_cm.rotmat, worklist.capacitor_cm.pos)
    print("relay____ rotmat is:\t pos is:")
    print(worklist.relay_205B_cm.rotmat, worklist.relay_205B_cm.pos)
    print("belt_____ rotmat is:\t pos is:")
    print(worklist.belt_cm.rotmat, worklist.belt_cm.pos)
    print("terminal_ rotmat is:\t pos is:")
    print(worklist.terminal_block_cm.rotmat, worklist.terminal_block_cm.pos)

    # workbench collision check
    print("workbench collision check")
    print("bracket collided?:", worklist.workbench_cm.is_mcdwith(worklist.bracketR1_cm))
    print("capacitor collided?:", worklist.workbench_cm.is_mcdwith(worklist.capacitor_cm))
    print("relay collided?:", worklist.workbench_cm.is_mcdwith(worklist.relay_205B_cm))
    print("belt collided?:", worklist.workbench_cm.is_mcdwith(worklist.belt_cm))
    print("terminal collided?:", worklist.workbench_cm.is_mcdwith(worklist.terminal_block_cm))

    # bracketR1 collision check
    print("\nbracketR1 collision check")
    print("workbench collided?:", worklist.bracketR1_cm.is_mcdwith(worklist.workbench_cm))
    print("capacitor collided?:", worklist.bracketR1_cm.is_mcdwith(worklist.capacitor_cm))
    print("relay collided?:", worklist.bracketR1_cm.is_mcdwith(worklist.relay_205B_cm))
    print("belt collided?:", worklist.bracketR1_cm.is_mcdwith(worklist.belt_cm))
    print("terminal collided?:", worklist.bracketR1_cm.is_mcdwith(worklist.terminal_block_cm))

    base.run()