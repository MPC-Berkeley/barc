function saveOldTraj(oldTraj::OldTrajectory,zCurr::Array{Float64},uCurr::Array{Float64},lapStatus::LapStatus,buffersize::Int64,dt::Float64)
                
                i               = lapStatus.currentIt           # current iteration number, just to make notation shorter
                zCurr_export    = zeros(buffersize,4)
                uCurr_export    = zeros(buffersize,2)
                zCurr_export    = cat(1,zCurr[1:i-1,:], [zCurr[i-1,1]+collect(1:buffersize-i+1)*dt*zCurr[i-1,4] ones(buffersize-i+1,1)*zCurr[i-1,2:4]])
                uCurr_export    = cat(1,uCurr[1:i-1,:], zeros(buffersize-i+1,2))
                costLap         = lapStatus.currentIt               # the cost of the current lap is the time it took to reach the finish line
                # Save all data in oldTrajectory:
                if lapStatus.currentLap == 1                        # if it's the first lap
                    oldTraj.oldTraj[:,:,1]  = zCurr_export         # ... just save everything
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldTraj[:,:,2]  = zCurr_export
                    oldTraj.oldInput[:,:,2] = uCurr_export
                    oldTraj.oldCost = [costLap,costLap]
                else                                                # idea: always copy the new trajectory in the first array!
                    if oldTraj.oldCost[1] < oldTraj.oldCost[2]      # if the first old traj is better than the second
                        oldTraj.oldTraj[:,:,2]  = oldTraj.oldTraj[:,:,1]    # ... copy the first in the second
                        oldTraj.oldInput[:,:,2] = oldTraj.oldInput[:,:,1]   # ... same for the input
                        oldTraj.oldCost[2] = oldTraj.oldCost[1]
                    end
                    oldTraj.oldTraj[:,:,1]  = zCurr_export                 # ... and write the new traj in the first
                    oldTraj.oldInput[:,:,1] = uCurr_export
                    oldTraj.oldCost[1] = costLap
                end
                #println(size(save_oldTraj))
                #println(size(oldTraj.oldTraj))
                #save_oldTraj[:,:,:,lapStatus.currentLap] = oldTraj.oldTraj[:,:,:]

                
end