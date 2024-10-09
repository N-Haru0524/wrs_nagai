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

    work_list = WORK_LIST()
    work_list.attach_to(base)

    # # load objects
    # # workbench
    # workbench_file = os.path.join(mesh.__path__[0], "workbench.stl")
    # workbench_cm = mcm.CollisionModel(initor=workbench_file,rgb=rm.const.orange_red)
    # workbench_cm.pos = rm.np.array([0,0,0])
    # workbench_cm.rotmat = rm.rotmat_from_euler(ai=rm.np.pi/2,aj=0,ak=0,order='rxyz')
    # workbench_cm.rotmat = rm.np.array([[1,0,0],[0,1,0],[0,0,1]])
    # workbench_cm.attach_to(base)

    # # bracketR1
    # bracketR1_file = os.path.join(mesh.__path__[0], "bracketR1.stl")
    # bracketR1_cm = mcm.CollisionModel(initor=bracketR1_file,rgb=rm.const.gray)
    # bracketR1_cm.pos = rm.np.array([-0.09,0,0.022])
    # bracketR1_cm.rotmat = rm.rotmat_from_euler(ai=rm.np.pi/2,aj=0,ak=0,order='rxyz')
    # bracketR1_cm.pos = rm.np.array([-0.09,0.022,0])
    # bracketR1_cm.rotmat = rm.np.array([[1,0,0],[0,1,0],[0,0,1]])
    # bracketR1_cm.attach_to(base)
    
    # # capacitor
    # capacitor_file = os.path.join(mesh.__path__[0], "capacitor.stl")
    # capacitor_cm = mcm.CollisionModel(initor=capacitor_file,rgb=rm.const.blue)
    # capacitor_cm.pos = rm.np.array([0.02,0,0.046])
    # capacitor_cm.rotmat = rm.rotmat_from_euler(ai=0,aj=-rm.np.pi/2,ak=0,order='rxyz')
    # capacitor_cm.pos = rm.np.array([0.02,0.046,0])
    # capacitor_cm.rotmat = rm.np.array([[0,0,-1],[1,0,0],[0,-1,0]])
    # capacitor_cm.attach_to(base)

    # # relay_205B
    # relay_205B_file = os.path.join(mesh.__path__[0], "relay_205B.stl")
    # relay_205B_cm = mcm.CollisionModel(initor=relay_205B_file,rgb=rm.const.black)
    # relay_205B_cm.pos = rm.np.array([0.091,-0.0225,-0.003])
    # relay_205B_cm.rotmat = rm.rotmat_from_euler(ai=rm.np.pi,aj=rm.np.pi/2,ak=0,order='rxyz')
    # relay_205B_cm.pos = rm.np.array([0.091,-0.003,0.0225])
    # relay_205B_cm.rotmat = rm.np.array([[0,0,1],[1,0,0],[0,1,0]])
    # relay_205B_cm.attach_to(base)

    # # belt
    # belt_file = os.path.join(mesh.__path__[0], "belt.stl")
    # belt_cm = mcm.CollisionModel(initor=belt_file,rgb=rm.const.deep_sky_blue)
    # belt_cm.pos = rm.np.array([-0.044,-0.027,0.05])
    # belt_cm.rotmat = rm.rotmat_from_euler(ai=rm.np.pi,aj=0,ak=0,order='rxyz')
    # belt_cm.pos = rm.np.array([-0.044,0.05,0.027])
    # belt_cm.rotmat = rm.np.array([[1,0,0],[0,0,-1],[0,1,0]])
    # belt_cm.attach_to(base)

    # # terminal_block
    # terminal_block_file = os.path.join(mesh.__path__[0], "terminal_block.stl")
    # terminal_block_cm = mcm.CollisionModel(initor=terminal_block_file,rgb=rm.const.yellow)
    # terminal_block_cm.pos = rm.np.array([0.065,0,0.023])
    # terminal_block_cm.rotmat = rm.rotmat_from_euler(ai=rm.np.pi/2,aj=rm.np.pi/2,ak=0,order='rxyz')
    # terminal_block_cm.pos = rm.np.array([0.065,0.023,0])
    # terminal_block_cm.rotmat = rm.np.array([[0,0,1],[0,1,0],[-1,0,0]])
    # terminal_block_cm.attach_to(base)

    # #numpy print options
    # rm.np.set_printoptions(precision=3,suppress=True)

    # inv = rm.np.linalg.inv(workbench_cm.rotmat)

    # #print rotmats and pos
    # print("workbench rotmat is:\t pos is:")
    # print(rm.np.dot(inv, workbench_cm.rotmat),rm.np.dot(inv, workbench_cm.pos))
    # print("bracketR1 rotmat is:\t pos is:")
    # print(rm.np.dot(inv, bracketR1_cm.rotmat),rm.np.dot(inv, bracketR1_cm.pos))
    # print("capacitor rotmat is:\t pos is:")
    # print(rm.np.dot(inv, capacitor_cm.rotmat),rm.np.dot(inv, capacitor_cm.pos))
    # print("relay____ rotmat is:\t pos is:")
    # print(rm.np.dot(inv, relay_205B_cm.rotmat),rm.np.dot(inv, relay_205B_cm.pos))
    # print("belt_____ rotmat is:\t pos is:")
    # print(rm.np.dot(inv, belt_cm.rotmat),rm.np.dot(inv, belt_cm.pos))
    # print("terminal_ rotmat is:\t pos is:")
    # print(rm.np.dot(inv, terminal_block_cm.rotmat),rm.np.dot(inv, terminal_block_cm.pos))

    # # workbench collision check
    # print("workbench collision check")
    # print("bracket collided?:", workbench_cm.is_mcdwith(bracketR1_cm))
    # print("capacitor collided?:", workbench_cm.is_mcdwith(capacitor_cm))
    # print("relay collided?:", workbench_cm.is_mcdwith(relay_205B_cm))
    # print("belt collided?:", workbench_cm.is_mcdwith(belt_cm))
    # print("terminal collided?:", workbench_cm.is_mcdwith(terminal_block_cm))

    # # bracketR1 collision check
    # print("\nbracketR1 collision check")
    # print("workbench collided?:", bracketR1_cm.is_mcdwith(workbench_cm))
    # print("capacitor collided?:", bracketR1_cm.is_mcdwith(capacitor_cm))
    # print("relay collided?:", bracketR1_cm.is_mcdwith(relay_205B_cm))
    # print("belt collided?:", bracketR1_cm.is_mcdwith(belt_cm))
    # print("terminal collided?:", bracketR1_cm.is_mcdwith(terminal_block_cm))

    base.run()