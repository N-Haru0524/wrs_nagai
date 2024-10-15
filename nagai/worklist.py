from wrs import wd, rm, mcm, mgm
import wrs.modeling.constant as const
import nagai as khi
import yaml
import os

class WORK_LIST():
    def __init__(self,
                 pos = rm.np.array([0,0,0]),
                 rotmat = rm.rotmat_from_euler(ai=rm.np.pi/2,aj=0,ak=0,order='rxyz'),
                 yamlpath = khi.__path__[0],
                 meshpath=khi.__path__[0],
                 cdprim_type=const.CDPrimType.AABB,
                 cdmesh_type=const.CDMeshType.DEFAULT,
                 model_type='collisionmodel'):
        with open(os.path.join(yamlpath, "object.yaml")) as f:
            objects = yaml.safe_load(f)
        # workbench
        workbench_file = os.path.join(meshpath, "meshes", objects['object1']['name'] + ".stl")
        self.workbench = mgm.GeometricModel(initor=workbench_file,rgb=rm.const.orange_red)
        if model_type == 'collisionmodel':
            self.workbench = mcm.CollisionModel(initor=self.workbench, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.workbench.pos = rm.np.array(objects['object1']['pos']) + pos
        self.workbench.rotmat = rm.np.dot(rm.np.array(objects['object1']['rotmat']), rotmat)

        rot_o2h = self.workbench.rotmat
        pos_o2h = self.workbench.pos

        # bracketR1
        bracketR1_file = os.path.join(meshpath, "meshes", objects['object2']['name'] + ".stl")
        self.bracketR1 = mgm.GeometricModel(initor=bracketR1_file,rgb=rm.const.gray)
        if model_type == 'collisionmodel':
            self.bracketR1 = mcm.CollisionModel(initor=self.bracketR1, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.bracketR1.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['pos'])) + pos_o2h
        self.bracketR1.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object2']['rotmat']))

        # capacitor
        capacitor_file = os.path.join(meshpath, "meshes", objects['object3']['name'] + ".stl")
        self.capacitor = mgm.GeometricModel(initor=capacitor_file,rgb=rm.const.blue)
        if model_type == 'collisionmodel':
            self.capacitor = mcm.CollisionModel(initor=self.capacitor, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.capacitor.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['pos'])) + pos_o2h
        self.capacitor.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object3']['rotmat']))

        # relay_205B
        relay_205B_file = os.path.join(meshpath, "meshes", objects['object4']['name'] + ".stl")
        self.relay_205B = mgm.GeometricModel(initor=relay_205B_file,rgb=rm.const.black)
        if model_type == 'collisionmodel':
            self.relay_205B = mcm.CollisionModel(initor=self.relay_205B, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.relay_205B.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['pos'])) + pos_o2h
        self.relay_205B.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object4']['rotmat']))

        # belt
        belt_file = os.path.join(meshpath, "meshes", objects['object5']['name'] + ".stl")
        self.belt = mgm.GeometricModel(initor=belt_file,rgb=rm.const.deep_sky_blue)
        if model_type == 'collisionmodel':
            self.belt = mcm.CollisionModel(initor=self.belt, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.belt.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['pos'])) + pos_o2h
        self.belt.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object5']['rotmat']))

        # terminal_block
        terminal_block_file = os.path.join(meshpath, "meshes", objects['object6']['name'] + ".stl")
        self.terminal_block = mgm.GeometricModel(initor=terminal_block_file,rgb=rm.const.yellow)
        if model_type == 'collisionmodel':
            self.terminal_block = mcm.CollisionModel(initor=self.terminal_block, cdprim_type=cdprim_type, cdmesh_type=cdmesh_type)
        self.terminal_block.pos = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['pos'])) + pos_o2h
        self.terminal_block.rotmat = rm.np.dot(rot_o2h, rm.np.array(objects['object6']['rotmat']))

    def init_pos(self, seed=0):
        rm.np.random.seed(seed)
        self.workbench.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])
        self.bracketR1.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])
        self.capacitor.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])
        self.relay_205B.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])
        self.belt.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])
        self.terminal_block.pos = rm.np.random.rand(3) / 2  + rm.np.array([-.5, -.25, .1])

    def init_rotmat(self, seed=0):
        rm.np.random.seed(seed)
        self.workbench.rotmat = rm.np.random.rand(3,3)
        self.bracketR1.rotmat = rm.np.random.rand(3,3)
        self.capacitor.rotmat = rm.np.random.rand(3,3)
        self.relay_205B.rotmat = rm.np.random.rand(3,3)
        self.belt.rotmat = rm.np.random.rand(3,3)
        self.terminal_block.rotmat = rm.np.random.rand(3,3)

    def attach_to(self, base):
        self.workbench.attach_to(base)
        self.bracketR1.attach_to(base)
        self.capacitor.attach_to(base)
        self.relay_205B.attach_to(base)
        self.belt.attach_to(base)
        self.terminal_block.attach_to(base)

if __name__ == '__main__':
    import wrs.modeling.constant as const
    class Data(object):
        def __init__(self, mot_data):
            self.counter = 0
            self.mot_data = mot_data

    base = wd.World(cam_pos=[.5,.5,.5], lookat_pos=[0, 0, 0])
    mgm.gen_frame(ax_length=.15).attach_to(base)

    worklist = WORK_LIST(pos=rm.np.array([0, 0, .12]))
    worklist.attach_to(base)

    ground = mcm.CollisionModel(initor=os.path.join(khi.__path__[0], "meshes", "base_table" + ".stl"), rgb=rm.const.blue)
    ground.pos = rm.np.array([-.59, 0, -.74])
    ground.attach_to(base)

    #numpy print options
    rm.np.set_printoptions(precision=3,suppress=True)
    
    #print rotmats and pos
    print("workbench rotmat is:\t pos is:")
    print(worklist.workbench.rotmat, worklist.workbench.pos)
    print("bracketR1 rotmat is:\t pos is:")
    print(worklist.bracketR1.rotmat, worklist.bracketR1.pos)
    print("capacitor rotmat is:\t pos is:")
    print(worklist.capacitor.rotmat, worklist.capacitor.pos)
    print("relay____ rotmat is:\t pos is:")
    print(worklist.relay_205B.rotmat, worklist.relay_205B.pos)
    print("belt_____ rotmat is:\t pos is:")
    print(worklist.belt.rotmat, worklist.belt.pos)
    print("terminal_ rotmat is:\t pos is:")
    print(worklist.terminal_block.rotmat, worklist.terminal_block.pos)

    # workbench collision check
    print("workbench collision check")
    print("bracket collided?:", worklist.workbench.is_mcdwith(worklist.bracketR1))
    print("capacitor collided?:", worklist.workbench.is_mcdwith(worklist.capacitor))
    print("relay collided?:", worklist.workbench.is_mcdwith(worklist.relay_205B))
    print("belt collided?:", worklist.workbench.is_mcdwith(worklist.belt))
    print("terminal collided?:", worklist.workbench.is_mcdwith(worklist.terminal_block))

    # bracketR1 collision check
    print("\nbracketR1 collision check")
    print("workbench collided?:", worklist.bracketR1.is_mcdwith(worklist.workbench))
    print("capacitor collided?:", worklist.bracketR1.is_mcdwith(worklist.capacitor))
    print("relay collided?:", worklist.bracketR1.is_mcdwith(worklist.relay_205B))
    print("belt collided?:", worklist.bracketR1.is_mcdwith(worklist.belt))
    print("terminal collided?:", worklist.bracketR1.is_mcdwith(worklist.terminal_block))

    # ground collision check
    print("\nground collision check")
    print("workbench collided?:", ground.is_mcdwith(worklist.workbench))

    base.run()