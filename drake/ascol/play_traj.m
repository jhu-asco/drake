function play_traj(xs, ts, S)
trj = DTTrajectory(ts, xs);
trj = setOutputFrame(trj,S.r.getStateFrame());
S.v.playback(trj);
end