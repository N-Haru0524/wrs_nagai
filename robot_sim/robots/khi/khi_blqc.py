import math
import numpy as np
import modeling.collision_model as mcm
import modeling.model_collection as mmc
import robot_sim._kinematics.jl as rkjl
import robot_sim._kinematics.jlchain as rkjlc
import robot_sim.manipulators.rs007l.rs007l as manipulator
import robot_sim.robots.single_arm_robot_interface as ai


class KHI_BLQC(ai.SglArmRobotInterface):
    """
    author: weiwei
    date: 20230826toyonaka
    """

    def __init__(self, pos=np.zeros(3), rotmat=np.eye(3), name="khi_g", enable_cc=True):
        super().__init__(pos=pos, rotmat=rotmat, name=name, enable_cc=enable_cc)
        # arm
        self.manipulator = manipulator.RS007L(pos=pos,
                                              rotmat=rotmat,
                                              home_conf=np.zeros(6),
                                              name='rs007l', enable_cc=False)
        # tool changer
        self.tool_changer = rkjl.Anchor(name=name + '_tool_changer', pos=self.manipulator.gl_flange_pos,
                                        rotmat=self.manipulator.gl_flange_rotmat, n_flange=1)
        self.tool_changer.loc_flange_pose_list[0][0] = np.array([0, 0, .0315])
        self.tool_changer.lnk_list[0].cmodel = mcm.gen_stick(self.tool_changer.pos,
                                                             # TODO: change to combined model, 20230806
                                                             self.tool_changer.gl_flange_pose_list[0][0],
                                                             radius=0.05,
                                                             # rgba=[.2, .2, .2, 1], rgb will be overwritten
                                                             type='rect',
                                                             n_sec=36)
        # tool
        self.end_effector = None
        # tool center point
        self.manipulator.tcp_jnt_id = -1
        self.manipulator.loc_tcp_pos = self.tool_changer.gl_flange_pose_list[0][0]
        self.manipulator.loc_tcp_rotmat = self.tool_changer.gl_flange_pose_list[0][1]
        # collision detection
        if self.cc is not None:
            self.setup_cc()

    def setup_cc(self):
        # tc
        tc0 = self.cc.add_cce(self.tool_changer.lnk_list[0])
        # manipulator
        mlb = self.cc.add_cce(self.manipulator.jlc.anchor.lnk_list[0])
        ml0 = self.cc.add_cce(self.manipulator.jlc.jnts[0].lnk)
        ml1 = self.cc.add_cce(self.manipulator.jlc.jnts[1].lnk)
        ml2 = self.cc.add_cce(self.manipulator.jlc.jnts[2].lnk)
        ml3 = self.cc.add_cce(self.manipulator.jlc.jnts[3].lnk)
        ml4 = self.cc.add_cce(self.manipulator.jlc.jnts[4].lnk)
        ml5 = self.cc.add_cce(self.manipulator.jlc.jnts[5].lnk)
        from_list = [tc0, ml3, ml4, ml5]
        into_list = [mlb, ml0]
        self.cc.set_cdpair_by_ids(from_list, into_list)

    def fix_to(self, pos, rotmat):
        self.pos = pos
        self.rotmat = rotmat
        self.manipulator.fix_to(pos=pos, rotmat=rotmat)
        self.tool_changer.fix_to(pos=self._manipulator.gl_flange_pos, rotmat=self._manipulator.gl_flange_rotmat)

    def change_jaw_width(self, jaw_width):
        return self.change_ee_values(ee_values=jaw_width)

    def get_jaw_width(self):
        return self.get_ee_values()


if __name__ == '__main__':
    import basis.robot_math as rm
    import visualization.panda.world as wd
    import modeling.geometric_model as gm
    import robot_sim.end_effectors.gripper.or2fg7.or2fg7 as org
    import robot_sim.end_effectors.single_contact.screw_driver.orsd.orsd as ors
    import motion.probabilistic.rrt_connect as rrtc

    base = wd.World(cam_pos=[1.7, 1.7, 1.7], lookat_pos=[0, 0, .3])
    gm.gen_frame().attach_to(base)
    robot = KHI_BLQC(enable_cc=True)
    rrtc_planner = rrtc.RRTConnect(robot)

    ee_g_pos = np.array([0, .8, .19])
    ee_g_rotmat = rm.rotmat_from_euler(0, np.radians(180), 0)
    ee_g = org.OR2FG7(pos=ee_g_pos,
                      rotmat=ee_g_rotmat,
                      coupling_offset_pos=np.array([0, 0, 0.0314]))
    ee_g_meshmodel = ee_g.gen_meshmodel()
    ee_g_meshmodel.attach_to(base)
    jv_attach_eeg = robot.ik(ee_g_pos, ee_g_rotmat)

    ee_sd_pos = np.array([.15, .8, .19])
    ee_sd_rotmat = rm.rotmat_from_euler(0, np.radians(180), np.radians(-90))
    ee_sd = ors.ORSD(pos=ee_sd_pos,
                     rotmat=ee_sd_rotmat,
                     coupling_offset_pos=np.array([0, 0, 0.0314]),
                     enable_cc=True)
    ee_sd_meshmodel = ee_g.gen_meshmodel()
    ee_sd_meshmodel.attach_to(base)
    jv_attach_eesd = robot.ik(ee_g_pos, ee_g_rotmat)

    goal_pos = np.array([.3, -.8, .19])
    goal_rotmat = rm.rotmat_from_euler(0, np.radians(90), 0)
    jv_goal = robot.ik(tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)

    base.run()

    robot_path_attach_ee_g = rrtc_planner.plan(start_conf=robot_s.get_jnt_values(),
                                               goal_conf=jv_attach_eeg,
                                               obstacle_list=[],
                                               ext_dist=.1,
                                               max_time=300)
    print(len(robot_path_attach_ee_g))

    # for jnt_values in robot_path:
    #     robot_s.fk(jnt_values)
    #     robot_s.gen_meshmodel().attach_to(base)
    # base.run()

    robot_path = robot_path_attach_ee_g

    counter = [0]
    robot_attached_list = []
    object_attached_list = []


    def update(robot_s, ee_g, ee_sd, robot_path, counter, robot_attached_list, object_attached_list, task):
        if counter[0] >= len(robot_path):
            counter[0] = 0
        if len(robot_attached_list) != 0:
            for robot_attached in robot_attached_list:
                robot_attached.detach()
            for object_attached in object_attached_list:
                object_attached.detach()
            robot_attached_list.clear()
            object_attached_list.clear()
        jnt_values = robot_path[counter[0]]
        robot_s.fk(jnt_values)
        robot_meshmodel = robot_s.gen_mesh_model()
        robot_meshmodel.attach_to(base)
        robot_attached_list.append(robot_meshmodel)
        counter[0] += 1
        print(counter[0])
        return task.again


    taskMgr.doMethodLater(0.01, update, 'udpate',
                          extraArgs=[robot_s, ee_g, ee_sd, robot_path, counter, robot_attached_list,
                                     object_attached_list],
                          appendTask=True)
    base.run()

    robot_s = KHI_BLQC(enable_cc=True)
    robot_s.gen_meshmodel(toggle_tcp_frame=True, toggle_jnt_frames=True).attach_to(base)

    ee_g_pos = np.array([0, .8, .19])
    ee_g_rotmat = rm.rotmat_from_euler(0, np.radians(180), 0)
    ee_g = org.OR2FG7(pos=ee_g_pos,
                      rotmat=ee_g_rotmat,
                      coupling_offset_pos=np.array([0, 0, 0.0314]), enable_cc=True)
    ee_g.gen_meshmodel().attach_to(base)

    ee_sd_pos = np.array([.15, .8, .19])
    ee_sd_rotmat = rm.rotmat_from_euler(0, np.radians(180), np.radians(-90))
    ee_sd = ors.ORSD(pos=ee_sd_pos,
                     rotmat=ee_sd_rotmat,
                     coupling_offset_pos=np.array([0, 0, 0.0314]),
                     enable_cc=True)
    ee_sd.gen_meshmodel().attach_to(base)

    jnt_values = robot_s.ik(ee_g_pos, ee_g_rotmat)
    robot_s.fk(jnt_values=jnt_values)
    robot_s_meshmodel = robot_s.gen_meshmodel(toggle_tcp_frame=True)
    robot_s_meshmodel.attach_to(base)

    robot_s.attach_tool(ee_g)
    goal_pos = np.array([.3, -.8, .19])
    goal_rotmat = rm.rotmat_from_euler(0, np.radians(90), 0)
    jnt_values = robot_s.ik(tgt_pos=goal_pos, tgt_rotmat=goal_rotmat)
    robot_s.fk(jnt_values=jnt_values)
    robot_s_meshmodel = robot_s.gen_meshmodel(toggle_tcp_frame=True)
    robot_s_meshmodel.attach_to(base)
    base.run()

    tgt_pos = np.array([.45, .2, .35])
    tgt_rotmat = rm.rotmat_from_axangle([0, 1, 0], math.pi * 2 / 3)
    gm.gen_frame(pos=tgt_pos, rotmat=tgt_rotmat).attach_to(base)
    # base.run()
    component_name = 'arm'
    jnt_values = robot_s.ik(component_name, tgt_pos, tgt_rotmat)
    robot_s.fk(component_name, jnt_values=jnt_values)
    robot_s_meshmodel = robot_s.gen_meshmodel(toggle_tcp_frame=True)
    robot_s_meshmodel.attach_to(base)
    # robot_s.show_cdprimit()
    robot_s.gen_stickmodel().attach_to(base)
    # tic = time.time()
    # result = robot_s.is_collided()
    # toc = time.time()
    # print(result, toc - tic)
    base.run()
