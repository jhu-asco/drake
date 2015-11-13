function us = fromLinearInterp(ups, tps, ts)
us = zeros(size(ups,1), length(ts));
tp_idx = 2;
for i = 1:length(ts)
    if(ts(i) > tps(tp_idx))
        tp_idx = tp_idx + 1;
    end
    us(:,i) = (ts(i) - tps(tp_idx-1))*(ups(tp_idx) - ups(tp_idx-1))/(tps(tp_idx) - tps(tp_idx-1)) + ups(tp_idx-1);
end
end