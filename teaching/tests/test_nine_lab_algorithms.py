"""Numerical contracts for the new teaching experiments; no ROS required."""
import importlib
import os
import math
import numpy as np
import pytest
from teaching.navigation import search,line_of_sight,reachable_frontiers,icp,load_map
from teaching.tabular import GridRobot,value_iteration,policy_iteration,q_update
from teaching.ros_contract import sample_laser,from_pose


def test_searches_find_valid_equal_cost_routes_and_reject_blocked_goals():
    grid=np.zeros((12,12),bool);grid[2:10,6]=True
    routes=[search(grid,(1,1),(10,10),name)[0] for name in ('bfs','dijkstra','astar')]
    assert len({len(route) for route in routes})==1
    for route in routes:
        assert route[0]==(1,1) and route[-1]==(10,10)
        assert all(line_of_sight(grid,a,b) for a,b in zip(route[:-1],route[1:]))
    with pytest.raises(ValueError):search(grid,(1,1),(5,6))
    grid[:,6]=True
    assert search(grid,(1,1),(10,10))[0]==[]


def test_theta_shortcuts_remain_clear_and_corner_contact_is_rejected():
    grid=np.zeros((15,15),bool);grid[4:12,7]=True
    route,_=search(grid,(2,2),(12,12),'theta')
    assert route and all(line_of_sight(grid,a,b) for a,b in zip(route[:-1],route[1:]))
    grid=np.zeros((3,3),bool);grid[0,1]=True
    assert not line_of_sight(grid,(0,0),(1,1))


def test_frontier_never_crosses_unknown_or_disconnected_obstacles():
    grid=np.full((8,8),-1);grid[1:4,1:4]=0;grid[5:7,5:7]=0
    frontier=reachable_frontiers(grid,(2,2))
    assert frontier and all(grid[p]==0 and p[0]<4 and p[1]<4 for p in frontier)
    assert (2,2) not in frontier


def test_icp_recovers_small_rigid_transform_and_rejects_no_overlap():
    target=np.random.default_rng(4).normal(size=(100,2));angle=.03
    exact=np.asarray([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    shift=np.array([.02,-.04]);source=(target-shift)@exact
    R,t,residual=icp(source,target)
    assert np.allclose(R,exact,atol=1e-6) and np.allclose(t,shift,atol=1e-6)
    assert residual[-1]<1e-6
    with pytest.raises(ValueError):icp(source+100,target)


def test_map_loader_handles_image_direction_relative_path_and_unknown(tmp_path):
    from PIL import Image
    Image.fromarray(np.array([[0,205],[254,0]],dtype=np.uint8)).save(tmp_path/'m.pgm')
    (tmp_path/'m.yaml').write_text('image: m.pgm\nresolution: 0.2\norigin: [1, 2, 0.4]\noccupied_thresh: 0.65\nfree_thresh: 0.196\nnegate: 0\nmode: trinary\n')
    grid,res,origin=load_map(tmp_path/'m.yaml')
    assert grid.tolist()==[[0,100],[100,-1]] and res==.2
    assert origin.tolist()==[1,2,.4]


def test_value_and_policy_iteration_agree():
    env=GridRobot();v,p,_=value_iteration(env);v2,p2,_=policy_iteration(env)
    assert np.allclose(v,v2,atol=1e-6)
    assert v[env.state(env.goal)]==0


def test_q_learning_terminal_mask_and_selected_entry():
    update=importlib.import_module(os.environ['ARC_TABULAR']).q_update if os.environ.get('ARC_TABULAR') else q_update
    q=np.array([[2.,0.],[4.,3.]])
    assert update(q,0,0,1.,1,False,alpha=.2,gamma=.9)==pytest.approx(4.6)
    assert q[0,0]==pytest.approx(2.52) and q[1].tolist()==[4,3]
    update(q,0,0,1.,1,True,alpha=1.,gamma=.9)
    assert q[0,0]==1


def test_scan_angles_invalid_data_and_goal_frame():
    angles=np.linspace(-math.pi,math.pi,360)
    ranges=3.+np.cos(angles)
    beams=sample_laser(ranges,angles[0],angles[1]-angles[0],.12,12.)
    assert beams[0]==pytest.approx(2.) and beams[12]==pytest.approx(4.,abs=.001)
    obs=from_pose(beams,0,0,math.pi/2,0,1,0,0)
    assert obs[25]==pytest.approx(0,abs=1e-6) and obs[26]==pytest.approx(1)
    ranges[10]=np.nan
    with pytest.raises(ValueError):sample_laser(ranges,angles[0],angles[1]-angles[0],.12,12.)


def test_double_dqn_selection_and_terminal_targets():
    torch=pytest.importorskip('torch')
    from teaching.deep_q import td_targets
    fn=importlib.import_module(os.environ['ARC_DQN_TARGET']).td_targets if os.environ.get('ARC_DQN_TARGET') else td_targets
    online=torch.tensor([[5.,4.],[5.,4.]])
    target=torch.tensor([[2.,6.],[2.,6.]])
    rewards=torch.tensor([1.,1.]);terminal=torch.tensor([0.,1.])
    assert torch.allclose(fn(rewards,terminal,online,target,gamma=.9,double=True),torch.tensor([2.8,1.]))
    assert torch.allclose(fn(rewards,terminal,online,target,gamma=.9,double=False),torch.tensor([6.4,1.]))


def test_returns_and_wrapped_pid_derivative():
    pytest.importorskip('torch')
    from teaching.policy_gradient import discounted_returns
    from starters.algorithms.control import PID
    fn=importlib.import_module(os.environ['ARC_RETURN']).discounted_returns if os.environ.get('ARC_RETURN') else discounted_returns
    assert fn([1,2,3],.9)==pytest.approx([5.23,4.7,3.])
    pid=PID(kp=0,kd=1,dt=1,output_limit=10)
    pid(math.radians(179));assert pid(math.radians(-179))==pytest.approx(math.radians(2))
