
import * as THREE from "./lib/three.module.js";
import {transpose, matmul2, euler_angle_to_rotate_matrix_3by3,normalizeAngle, intersect } from "./util.js";
import { logger } from "./log.js";

// todo: this module needs a proper name

function AutoAdjust(boxOp, mouse, header){
    this.boxOp = boxOp,
    this.mouse = mouse;
    this.header = header;
    var marked_object = null;
    this.staticPropagationDefaults = {
        minPointsForStaticPropagation: 5,
        minPointsForPropagationRefine: 8,
        minPointsForPropagationDirectionStop: 3,
        debugPropagationTrackIds: [248, 249],
        maxStaticPropagationRefineShift: 1.0,
        maxPropagationRefineShiftXY: 0.6,
        maxPropagationRefineShiftZ: 0.35,
        minRefinedPointCountRetentionRatio: 0.6,
        propagatedRefineGrowBoxDistanceThreshold: 0.15,
        propagatedRefineInitScaleRatio: {x: 1.3, y: 1.3, z: 1.8},
        maxPropagationContinuitySpan: 12,
        maxPropagationSingleNeighborGap: 3,
        maxPropagationContinuityPositionError: 1.2,
        maxPropagationContinuityYawError: Math.PI/8,
        maxPropagationSingleFramePull: 0.75,
        maxPropagationSevereOverlapCenterDistance: 1.0,
        maxPropagationSevereOverlapYawDiff: Math.PI/4,
        maxPropagationSevereOverlapScaleRatioDiff: 0.35,
        maxStaticWorldPositionDrift: 1.5,
        maxStaticWorldPositionPull: 0.75,
        maxStaticWorldPositionDiscard: 3.0,
        maxStaticWorldYawDrift: Math.PI/12,
        maxStaticWorldYawDiscard: Math.PI/6,
        maxEgoYawDeltaForStaticHeadingCheck: Math.PI/18,
    };

    this.getStaticPropagationConfig = function(name){
        let value = pointsGlobalConfig ? pointsGlobalConfig[name] : undefined;
        if (value === undefined || value === null){
            return this.staticPropagationDefaults[name];
        }

        return value;
    };

    this.shouldDebugPropagation = function(trackId){
        let ids = this.getStaticPropagationConfig("debugPropagationTrackIds");
        if (!ids){
            return false;
        }

        if (!Array.isArray(ids)){
            ids = [ids];
        }

        return ids.map(id=>String(id)).includes(String(trackId));
    };

    this.debugPropagation = function(trackId, stage, payload){
        if (!this.shouldDebugPropagation(trackId)){
            return;
        }

        console.log(`[prop-debug ${trackId}] ${stage}`, payload);
    };

    this.summarizeBoxState = function(box){
        if (!box){
            return null;
        }

        return {
            frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
            annotator: box.annotator || "",
            position: {
                x: Number(box.position.x.toFixed(3)),
                y: Number(box.position.y.toFixed(3)),
                z: Number(box.position.z.toFixed(3)),
            },
            rotation: {
                x: Number(box.rotation.x.toFixed(3)),
                y: Number(box.rotation.y.toFixed(3)),
                z: Number(box.rotation.z.toFixed(3)),
            },
            scale: {
                x: Number(box.scale.x.toFixed(3)),
                y: Number(box.scale.y.toFixed(3)),
                z: Number(box.scale.z.toFixed(3)),
            },
        };
    };

    this.summarizeWorldState = function(worldState){
        if (!worldState){
            return null;
        }

        return {
            position: {
                x: Number(worldState.position.x.toFixed(3)),
                y: Number(worldState.position.y.toFixed(3)),
                z: Number(worldState.position.z.toFixed(3)),
            },
            rotation: {
                x: Number(worldState.rotation.x.toFixed(3)),
                y: Number(worldState.rotation.y.toFixed(3)),
                z: Number(worldState.rotation.z.toFixed(3)),
            },
        };
    };

    this.summarizeFilterMetrics = function(metrics){
        if (!metrics){
            return null;
        }

        return {
            frame: metrics.frame || null,
            direction: metrics.direction || 0,
            supportCount: metrics.supportCount || 0,
            coarsePointCount: metrics.coarsePointCount ?? null,
            refinedPointCount: metrics.refinedPointCount ?? null,
            finalPointCount: metrics.finalPointCount ?? null,
            overlapWith: metrics.overlapWith || null,
            keep: metrics.keep,
            reason: metrics.reason || "",
        };
    };

    this.getFrameDirection = function(anchorFrameIndex, targetFrameIndex){
        if (targetFrameIndex < anchorFrameIndex){
            return -1;
        }
        if (targetFrameIndex > anchorFrameIndex){
            return 1;
        }
        return 0;
    };

    this.shouldStopPropagationDirection = function(pointCount, expectedState){
        let minPoints = this.getStaticPropagationConfig("minPointsForPropagationDirectionStop") || 0;
        return !expectedState && pointCount <= minPoints;
    };

    this.getScaleRatioDiff = function(a, b){
        if (!a || !b){
            return 1;
        }

        let dx = Math.abs(a.x - b.x) / Math.max(a.x, b.x, 1e-6);
        let dy = Math.abs(a.y - b.y) / Math.max(a.y, b.y, 1e-6);
        let dz = Math.abs(a.z - b.z) / Math.max(a.z, b.z, 1e-6);
        return Math.max(dx, dy, dz);
    };

    this.getSevereOverlapInfo = function(targetBox){
        if (!targetBox || !targetBox.world || !targetBox.world.annotation){
            return null;
        }

        let maxCenterDistance = this.getStaticPropagationConfig("maxPropagationSevereOverlapCenterDistance") || 0;
        let maxYawDiff = this.getStaticPropagationConfig("maxPropagationSevereOverlapYawDiff") || 0;
        let maxScaleDiff = this.getStaticPropagationConfig("maxPropagationSevereOverlapScaleRatioDiff") || 0;
        let candidates = targetBox.world.annotation.boxes
            .filter(b=>b !== targetBox)
            .filter(b=>b.obj_track_id != targetBox.obj_track_id)
            .filter(b=>b.obj_type == targetBox.obj_type)
            .filter(b=>intersect(targetBox, b))
            .map(b=>{
                let centerDistance = this.distanceBetweenPositions(targetBox.position, b.position);
                let yawDiff = Math.abs(normalizeAngle(targetBox.rotation.z - b.rotation.z));
                let scaleDiff = this.getScaleRatioDiff(targetBox.scale, b.scale);
                return {
                    obj_track_id: b.obj_track_id,
                    annotator: b.annotator || "",
                    centerDistance,
                    yawDiff,
                    scaleDiff,
                    boxState: this.summarizeBoxState(b),
                };
            })
            .filter(info=>info.centerDistance <= maxCenterDistance && info.yawDiff <= maxYawDiff && info.scaleDiff <= maxScaleDiff)
            .sort((a, b)=>a.centerDistance - b.centerDistance);

        return candidates.length > 0 ? candidates[0] : null;
    };

    this.isStaticTarget = function(box){
        return !!(box && box.obj_attr && box.obj_attr.search('static') >= 0);
    };

    this.getBoxPointCount = function(box){
        if (!box || !box.world || !box.world.lidar){
            return 0;
        }

        return box.world.lidar.get_box_points_number(box);
    };

    this.getWorldStateFromBox = function(box){
        if (!box){
            return null;
        }

        return this.getStaticWorldState(box.world, box);
    };

    this.applyWorldStateToBox = function(box, worldState){
        if (!box || !box.world || !worldState){
            return;
        }

        let correctedPos = this.utmPosToLidar(box.world, worldState.position);
        let correctedRot = box.world.utmRotToLidar(worldState.rotation);
        box.position.x = correctedPos.x;
        box.position.y = correctedPos.y;
        box.position.z = correctedPos.z;
        box.rotation.x = correctedRot.x;
        box.rotation.y = correctedRot.y;
        box.rotation.z = correctedRot.z;
    };

    this.getPropagationWorldList = function(anchorWorld){
        if (!anchorWorld || !anchorWorld.data){
            return [];
        }

        let anchorFrameIndex = anchorWorld.frameInfo.frame_index;
        return anchorWorld.data.worldList
            .filter(w=>w.frameInfo.scene === anchorWorld.frameInfo.scene)
            .slice()
            .sort((a, b)=>{
                let da = Math.abs(a.frameInfo.frame_index - anchorFrameIndex);
                let db = Math.abs(b.frameInfo.frame_index - anchorFrameIndex);
                if (da !== db){
                    return da - db;
                }

                return a.frameInfo.frame_index - b.frameInfo.frame_index;
            });
    };

    this.interpolateAngle = function(start, end, ratio){
        return normalizeAngle(start + normalizeAngle(end - start) * ratio);
    };

    this.findContinuityNeighbor = function(worlds, trackId, targetFrameIndex, direction, preferManual){
        let best = null;
        worlds.forEach(w=>{
            let frameIndex = w.frameInfo.frame_index;
            if (direction < 0 && frameIndex >= targetFrameIndex){
                return;
            }
            if (direction > 0 && frameIndex <= targetFrameIndex){
                return;
            }

            let box = w.annotation.boxes.find(b=>b.obj_track_id == trackId);
            if (!box){
                return;
            }

            if (preferManual && box.annotator){
                return;
            }

            if (!best){
                best = {world: w, box, frameIndex};
                return;
            }

            let currentGap = Math.abs(frameIndex - targetFrameIndex);
            let bestGap = Math.abs(best.frameIndex - targetFrameIndex);
            if (currentGap < bestGap){
                best = {world: w, box, frameIndex};
            }
        });

        return best;
    };

    this.getContinuitySupport = function(propagationWorlds, trackId, targetWorld){
        let targetFrameIndex = targetWorld.frameInfo.frame_index;
        let worlds = propagationWorlds.filter(w=>w !== targetWorld);
        let prev = this.findContinuityNeighbor(worlds, trackId, targetFrameIndex, -1, true) ||
            this.findContinuityNeighbor(worlds, trackId, targetFrameIndex, -1, false);
        let next = this.findContinuityNeighbor(worlds, trackId, targetFrameIndex, 1, true) ||
            this.findContinuityNeighbor(worlds, trackId, targetFrameIndex, 1, false);
        return {prev, next};
    };

    this.getExpectedContinuityState = function(trackId, targetWorld, propagationWorlds){
        let support = this.getContinuitySupport(propagationWorlds, trackId, targetWorld);
        let targetFrameIndex = targetWorld.frameInfo.frame_index;
        let maxSpan = this.getStaticPropagationConfig("maxPropagationContinuitySpan") || 0;
        let maxSingleGap = this.getStaticPropagationConfig("maxPropagationSingleNeighborGap") || 0;

        if (support.prev && support.next){
            let span = support.next.frameIndex - support.prev.frameIndex;
            if (span > 0 && (!maxSpan || span <= maxSpan)){
                let prevState = this.getWorldStateFromBox(support.prev.box);
                let nextState = this.getWorldStateFromBox(support.next.box);
                if (!prevState || !nextState){
                    return null;
                }

                let ratio = (targetFrameIndex - support.prev.frameIndex) / span;
                return {
                    supportCount: 2,
                    position: {
                        x: prevState.position.x + (nextState.position.x - prevState.position.x) * ratio,
                        y: prevState.position.y + (nextState.position.y - prevState.position.y) * ratio,
                        z: prevState.position.z + (nextState.position.z - prevState.position.z) * ratio,
                    },
                    rotation: {
                        x: prevState.rotation.x + (nextState.rotation.x - prevState.rotation.x) * ratio,
                        y: prevState.rotation.y + (nextState.rotation.y - prevState.rotation.y) * ratio,
                        z: this.interpolateAngle(prevState.rotation.z, nextState.rotation.z, ratio),
                    }
                };
            }
        }

        let singleSupport = support.prev || support.next;
        if (!singleSupport || !maxSingleGap){
            return null;
        }

        if (Math.abs(singleSupport.frameIndex - targetFrameIndex) > maxSingleGap){
            return null;
        }

        let singleState = this.getWorldStateFromBox(singleSupport.box);
        if (!singleState){
            return null;
        }

        return {
            supportCount: 1,
            position: singleState.position,
            rotation: singleState.rotation,
        };
    };

    this.applyContinuityConstraint = function(targetBox, trackId, propagationWorlds, expectedState){
        if (!targetBox){
            return {expectedState: null};
        }

        expectedState = expectedState || this.getExpectedContinuityState(trackId, targetBox.world, propagationWorlds);
        if (!expectedState){
            this.debugPropagation(trackId, "continuity skipped", {
                frame: targetBox.world.frameInfo.frame,
                reason: "no support",
            });
            return {expectedState: null};
        }

        let currentState = this.getWorldStateFromBox(targetBox);
        if (!currentState){
            this.debugPropagation(trackId, "continuity skipped", {
                frame: targetBox.world.frameInfo.frame,
                reason: "no current world state",
            });
            return {expectedState};
        }

        let positionError = this.distanceBetweenPositions(currentState.position, expectedState.position);
        let maxPositionError = this.getStaticPropagationConfig("maxPropagationContinuityPositionError") || 0;
        this.debugPropagation(trackId, "continuity check", {
            frame: targetBox.world.frameInfo.frame,
            supportCount: expectedState.supportCount,
            currentState: this.summarizeWorldState(currentState),
            expectedState: this.summarizeWorldState(expectedState),
            positionError: Number(positionError.toFixed(3)),
            maxPositionError,
        });
        if (maxPositionError > 0 && positionError > maxPositionError){
            if (expectedState.supportCount > 1){
                let correctedState = {
                    position: expectedState.position,
                    rotation: currentState.rotation,
                };
                this.applyWorldStateToBox(targetBox, correctedState);
                console.log(`clamp propagated box ${trackId} in ${targetBox.world.frameInfo.frame}: continuity position error ${positionError.toFixed(3)}m`);
            } else{
                let maxPull = this.getStaticPropagationConfig("maxPropagationSingleFramePull") || 0;
                if (maxPull > 0){
                    let pullDistance = Math.min(positionError - maxPositionError, maxPull);
                    let correctedState = {
                        position: {
                            x: currentState.position.x + (expectedState.position.x - currentState.position.x) * pullDistance / positionError,
                            y: currentState.position.y + (expectedState.position.y - currentState.position.y) * pullDistance / positionError,
                            z: currentState.position.z + (expectedState.position.z - currentState.position.z) * pullDistance / positionError,
                        },
                        rotation: currentState.rotation,
                    };
                    this.applyWorldStateToBox(targetBox, correctedState);
                    console.log(`pull propagated box ${trackId} in ${targetBox.world.frameInfo.frame}: continuity position error ${positionError.toFixed(3)}m`);
                }
            }
        }

        currentState = this.getWorldStateFromBox(targetBox);
        if (!currentState){
            return {expectedState};
        }

        let maxYawError = this.getStaticPropagationConfig("maxPropagationContinuityYawError") || 0;
        let yawError = Math.abs(normalizeAngle(currentState.rotation.z - expectedState.rotation.z));
        if (maxYawError > 0 && yawError > maxYawError){
            let correctedState = {
                position: currentState.position,
                rotation: {
                    x: currentState.rotation.x,
                    y: currentState.rotation.y,
                    z: expectedState.supportCount > 1
                        ? expectedState.rotation.z
                        : normalizeAngle(currentState.rotation.z + Math.sign(normalizeAngle(expectedState.rotation.z - currentState.rotation.z)) * maxYawError),
                }
            };
            this.applyWorldStateToBox(targetBox, correctedState);
            console.log(`clamp propagated heading ${trackId} in ${targetBox.world.frameInfo.frame}: continuity yaw error ${(yawError*180/Math.PI).toFixed(2)}deg`);
        }

        return {expectedState};
    };

    this.refinePropagatedBox = async function(box){
        if (!box || !box.world || !box.world.lidar){
            return {coarsePointCount: 0, refinedPointCount: 0, skipped: true};
        }

        let coarsePointCount = this.getBoxPointCount(box);
        let minRefinePoints = this.getStaticPropagationConfig("minPointsForPropagationRefine") || 0;
        if (minRefinePoints > 0 && coarsePointCount < minRefinePoints){
            this.debugPropagation(box.obj_track_id, "refine skipped", {
                frame: box.world.frameInfo.frame,
                coarsePointCount,
                minRefinePoints,
                boxState: this.summarizeBoxState(box),
            });
            console.log(`skip propagated refine ${box.obj_track_id} in ${box.world.frameInfo.frame}: ${coarsePointCount} < ${minRefinePoints}`);
            return {coarsePointCount, refinedPointCount: coarsePointCount, skipped: true};
        }

        let savedGrowThreshold = this.boxOp.grow_box_distance_threshold;
        let savedInitScaleRatio = {
            x: this.boxOp.init_scale_ratio.x,
            y: this.boxOp.init_scale_ratio.y,
            z: this.boxOp.init_scale_ratio.z,
        };

        try{
            let localGrowThreshold = this.getStaticPropagationConfig("propagatedRefineGrowBoxDistanceThreshold");
            let localInitScaleRatio = this.getStaticPropagationConfig("propagatedRefineInitScaleRatio");
            if (localGrowThreshold){
                this.boxOp.grow_box_distance_threshold = localGrowThreshold;
            }
            if (localInitScaleRatio){
                this.boxOp.init_scale_ratio = {
                    x: localInitScaleRatio.x,
                    y: localInitScaleRatio.y,
                    z: localInitScaleRatio.z,
                };
            }
            await this.boxOp.auto_rotate_xyz(box, null, null, null, true, null);
        }
        catch (error){
            console.error("auto fit propagated box failed", error);
        }
        finally{
            this.boxOp.grow_box_distance_threshold = savedGrowThreshold;
            this.boxOp.init_scale_ratio = savedInitScaleRatio;
        }

        let refineResult = {
            coarsePointCount,
            refinedPointCount: this.getBoxPointCount(box),
            skipped: false,
        };
        this.debugPropagation(box.obj_track_id, "refine finished", {
            frame: box.world.frameInfo.frame,
            coarsePointCount: refineResult.coarsePointCount,
            refinedPointCount: refineResult.refinedPointCount,
            boxState: this.summarizeBoxState(box),
        });
        return refineResult;
    };

    this.shouldRestoreCoarseAfterRefine = function(targetBox, coarseState, refineResult){
        if (!targetBox || !coarseState){
            return {restore: false};
        }

        let dx = targetBox.position.x - coarseState.position.x;
        let dy = targetBox.position.y - coarseState.position.y;
        let dz = Math.abs(targetBox.position.z - coarseState.position.z);
        let xyShift = Math.sqrt(dx*dx + dy*dy);
        let maxXYShift = this.getStaticPropagationConfig("maxPropagationRefineShiftXY") || 0;
        if (maxXYShift > 0 && xyShift > maxXYShift){
            return {restore: true, reason: `refine xy shift ${xyShift.toFixed(3)}m`};
        }

        let maxZShift = this.getStaticPropagationConfig("maxPropagationRefineShiftZ") || 0;
        if (maxZShift > 0 && dz > maxZShift){
            return {restore: true, reason: `refine z shift ${dz.toFixed(3)}m`};
        }

        let refineShiftLimit = this.getStaticPropagationConfig("maxStaticPropagationRefineShift") || 0;
        if (refineShiftLimit > 0){
            let refineShift = this.distanceBetweenPositions(targetBox.position, coarseState.position);
            if (refineShift > refineShiftLimit){
                return {restore: true, reason: `refine shift ${refineShift.toFixed(3)}m`};
            }
        }

        if (refineResult && !refineResult.skipped){
            let minRetentionRatio = this.getStaticPropagationConfig("minRefinedPointCountRetentionRatio") || 0;
            if (minRetentionRatio > 0 && refineResult.coarsePointCount > 0){
                let retainedRatio = refineResult.refinedPointCount / refineResult.coarsePointCount;
                if (retainedRatio < minRetentionRatio){
                    return {
                        restore: true,
                        reason: `refined point retention ${retainedRatio.toFixed(2)} < ${minRetentionRatio.toFixed(2)}`
                    };
                }
            }
        }

        return {restore: false};
    };

    this.cloneBoxState = function(box){
        return {
            position: {x: box.position.x, y: box.position.y, z: box.position.z},
            rotation: {x: box.rotation.x, y: box.rotation.y, z: box.rotation.z},
            scale: {x: box.scale.x, y: box.scale.y, z: box.scale.z},
        };
    };

    this.applyBoxState = function(box, state){
        box.position.x = state.position.x;
        box.position.y = state.position.y;
        box.position.z = state.position.z;
        box.rotation.x = state.rotation.x;
        box.rotation.y = state.rotation.y;
        box.rotation.z = state.rotation.z;
        box.scale.x = state.scale.x;
        box.scale.y = state.scale.y;
        box.scale.z = state.scale.z;
    };

    this.distanceBetweenPositions = function(a, b){
        if (!a || !b){
            return 0;
        }

        let dx = a.x - b.x;
        let dy = a.y - b.y;
        let dz = a.z - b.z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    };

    this.getStaticWorldState = function(world, box){
        if (!world || !box || !world.lidarPosToUtm || !world.lidarRotToUtm){
            return null;
        }

        let utmPosition = world.lidarPosToUtm(box.position);
        let utmRotation = world.lidarRotToUtm(box.rotation);
        return {
            position: {x: utmPosition.x, y: utmPosition.y, z: utmPosition.z},
            rotation: {x: utmRotation.x, y: utmRotation.y, z: normalizeAngle(utmRotation.z)},
        };
    };

    this.utmPosToLidar = function(world, pos){
        let lidarPos = new THREE.Vector4(pos.x, pos.y, pos.z, 1).applyMatrix4(world.trans_utm_lidar);
        return {x: lidarPos.x, y: lidarPos.y, z: lidarPos.z};
    };

    this.getEgoYaw = function(world){
        let pose = world && world.egoPose && world.egoPose.egoPose;
        let azimuth = pose && (pose.azimuth ?? pose.yaw);
        if (azimuth === undefined || azimuth === null){
            return null;
        }

        return normalizeAngle(-azimuth * Math.PI / 180.0);
    };

    this.shouldCheckStaticHeading = function(anchorWorld, targetWorld){
        let maxYawDelta = this.getStaticPropagationConfig("maxEgoYawDeltaForStaticHeadingCheck") || 0;
        if (maxYawDelta <= 0){
            return true;
        }

        let anchorYaw = this.getEgoYaw(anchorWorld);
        let targetYaw = this.getEgoYaw(targetWorld);
        if (anchorYaw === null || targetYaw === null){
            return false;
        }

        return Math.abs(normalizeAngle(targetYaw - anchorYaw)) <= maxYawDelta;
    };

    this.correctStaticPropagationDrift = function(targetBox, anchorWorldState, anchorWorld){
        let correction = {discard: false};
        if (!targetBox || !anchorWorldState){
            return correction;
        }

        let world = targetBox.world;
        let targetWorldState = this.getStaticWorldState(world, targetBox);
        if (!targetWorldState){
            return correction;
        }

        let maxPositionDrift = this.getStaticPropagationConfig("maxStaticWorldPositionDrift") || 0;
        if (maxPositionDrift > 0){
            let drift = this.distanceBetweenPositions(targetWorldState.position, anchorWorldState.position);
            if (drift > maxPositionDrift){
                let discardDrift = this.getStaticPropagationConfig("maxStaticWorldPositionDiscard") || 0;
                if (discardDrift > 0 && drift > discardDrift){
                    this.debugPropagation(targetBox.obj_track_id, "static drift discard", {
                        frame: world.frameInfo.frame,
                        drift: Number(drift.toFixed(3)),
                        discardDrift,
                        anchorWorldState: this.summarizeWorldState(anchorWorldState),
                        targetWorldState: this.summarizeWorldState(targetWorldState),
                    });
                    console.log(`discard static box ${targetBox.obj_track_id} in ${world.frameInfo.frame}: world drift ${drift.toFixed(3)}m`);
                    correction.discard = true;
                    return correction;
                }

                let maxPull = this.getStaticPropagationConfig("maxStaticWorldPositionPull") || 0;
                let pullDistance = Math.min(drift - maxPositionDrift, maxPull);
                if (pullDistance > 0){
                    let correctedWorldPos = {
                        x: targetWorldState.position.x + (anchorWorldState.position.x - targetWorldState.position.x) * pullDistance / drift,
                        y: targetWorldState.position.y + (anchorWorldState.position.y - targetWorldState.position.y) * pullDistance / drift,
                        z: targetWorldState.position.z + (anchorWorldState.position.z - targetWorldState.position.z) * pullDistance / drift,
                    };
                    let correctedPos = this.utmPosToLidar(world, correctedWorldPos);
                    targetBox.position.x = correctedPos.x;
                    targetBox.position.y = correctedPos.y;
                    targetBox.position.z = correctedPos.z;
                    console.log(`pull back static box ${targetBox.obj_track_id} in ${world.frameInfo.frame}: world drift ${drift.toFixed(3)}m, pull ${pullDistance.toFixed(3)}m`);
                    targetWorldState = this.getStaticWorldState(world, targetBox);
                }
            }
        }

        let maxYawDrift = this.getStaticPropagationConfig("maxStaticWorldYawDrift") || 0;
        if (maxYawDrift > 0 && this.shouldCheckStaticHeading(anchorWorld, world)){
            let yawDelta = normalizeAngle(targetWorldState.rotation.z - anchorWorldState.rotation.z);
            let yawDrift = Math.abs(yawDelta);
            if (yawDrift > maxYawDrift){
                let discardYawDrift = this.getStaticPropagationConfig("maxStaticWorldYawDiscard") || 0;
                if (discardYawDrift > 0 && yawDrift > discardYawDrift){
                    console.log(`discard static heading ${targetBox.obj_track_id} in ${world.frameInfo.frame}: world yaw drift ${(yawDrift*180/Math.PI).toFixed(2)}deg`);
                    correction.discard = true;
                    return correction;
                }

                let correctedWorldRot = {
                    x: targetWorldState.rotation.x,
                    y: targetWorldState.rotation.y,
                    z: normalizeAngle(anchorWorldState.rotation.z + Math.sign(yawDelta) * maxYawDrift),
                };
                let correctedRot = world.utmRotToLidar(correctedWorldRot);
                targetBox.rotation.x = correctedRot.x;
                targetBox.rotation.y = correctedRot.y;
                targetBox.rotation.z = correctedRot.z;
                console.log(`clamp static heading ${targetBox.obj_track_id} in ${world.frameInfo.frame}: world yaw drift ${(yawDrift*180/Math.PI).toFixed(2)}deg`);
            }
        }

        return correction;
    };

    this.shouldKeepStaticPropagationBox = function(box){
        let result = {
            keep: true,
            reason: "",
            frame: null,
            direction: 0,
            supportCount: 0,
            coarsePointCount: null,
            refinedPointCount: null,
            pointCount: null,
            finalPointCount: null,
            overlapWith: null,
        };

        if (!box || !box.world || !box.world.lidar){
            return result;
        }

        let severeOverlap = this.getSevereOverlapInfo(box);
        if (severeOverlap){
            result.keep = false;
            result.reason = "severe_overlap";
            result.overlapWith = {
                obj_track_id: severeOverlap.obj_track_id,
                annotator: severeOverlap.annotator,
                centerDistance: Number(severeOverlap.centerDistance.toFixed(3)),
                yawDiffDeg: Number((severeOverlap.yawDiff * 180 / Math.PI).toFixed(2)),
                scaleDiff: Number(severeOverlap.scaleDiff.toFixed(3)),
                boxState: severeOverlap.boxState,
            };
            console.log(`reject overlapped propagated box ${box.obj_track_id} in ${box.world.frameInfo.frame}: overlaps ${severeOverlap.obj_track_id}, center ${severeOverlap.centerDistance.toFixed(3)}m`);
            return result;
        }

        let minPoints = this.getStaticPropagationConfig("minPointsForStaticPropagation") || 0;
        result.pointCount = box.world.lidar.get_box_points_number(box);
        if (minPoints > 0 && result.pointCount < minPoints){
            result.keep = false;
            result.reason = "sparse_points";
            console.log(`skip sparse propagated box ${box.obj_track_id} in ${box.world.frameInfo.frame}: ${result.pointCount} < ${minPoints}`);
            return result;
        }

        return result;
    };

    this.discardPropagatedBox = function(box){
        if (!box || !box.world){
            return;
        }

        box.world.annotation.unload_box(box);
        box.world.annotation.remove_box(box);
    };

    // mark bbox, which will be used as reference-bbox of an object.
    this.mark_bbox=function(box){
        if (box){
            this.marked_object = {
                frame: box.world.frameInfo.frame,
                scene: box.world.frameInfo.scene,
                ann: box.world.annotation.boxToAnn(box),
            }
    
            logger.log(`selected reference objcet ${this.marked_object}`);
    
            this.header.set_ref_obj(this.marked_object);
        }
    };
    
    this.followStaticObjects = async function(box) {
        let world = box.world;
        let anchorFrameIndex = world.frameInfo.frame_index;
        let anchorWorldState = this.getStaticWorldState(world, box);
        let stoppedDirections = {
            "-1": false,
            "1": false,
        };
        let staticObjects = world.annotation.boxes.
            filter(b=>b!=box && b.obj_attr && b.obj_attr.search('static')>=0).
            map(refObj=>{
                let coord = euler_angle_to_rotate_matrix_3by3(refObj.rotation);
                let trans = transpose(coord, 3);
                let p = [box.position.x - refObj.position.x, 
                        box.position.y - refObj.position.y, 
                        box.position.z - refObj.position.z];
                let relativePos = matmul2(trans, p, 3);
                let relativeRot = {
                    x: normalizeAngle(box.rotation.x - refObj.rotation.x),
                    y: normalizeAngle(box.rotation.y - refObj.rotation.y),
                    z: normalizeAngle(box.rotation.z - refObj.rotation.z),
                };

                
                let distance = Math.sqrt(relativePos[0]*relativePos[0] + relativePos[1]*relativePos[1] + relativePos[2]*relativePos[2]);
                return {
                    obj_track_id: refObj.obj_track_id,
                    relativePos,
                    relativeRot,
                    distance
                    
                }
        });

        let targetIsStatic = this.isStaticTarget(box);
        let worldList = this.getPropagationWorldList(box.world);
        this.debugPropagation(box.obj_track_id, "follow static start", {
            anchorFrame: world.frameInfo.frame,
            targetIsStatic,
            targetBox: this.summarizeBoxState(box),
            anchorWorldState: this.summarizeWorldState(anchorWorldState),
            refCount: staticObjects.length,
            refs: staticObjects.map(ref=>({
                obj_track_id: ref.obj_track_id,
                distance: Number(ref.distance.toFixed(3)),
            })),
            worldListLength: worldList.length,
        });
            //let saveList = [];
        for (const w of worldList){
            if (w === box.world){
                //current frame
                continue;
            }

            let direction = this.getFrameDirection(anchorFrameIndex, w.frameInfo.frame_index);
            if (direction !== 0 && stoppedDirections[String(direction)]){
                this.debugPropagation(box.obj_track_id, "skip stopped direction", {
                    frame: w.frameInfo.frame,
                    direction,
                });
                continue;
            }
            
            let existedBox = w.annotation.boxes.find(b=>b.obj_track_id == box.obj_track_id);            
            if (existedBox && !existedBox.annotator)
            {
                // have same objects annotated.
                // if its generated by machine, lets overwrite it
                this.debugPropagation(box.obj_track_id, "skip existing manual box", this.summarizeBoxState(existedBox));
                continue;
            }

            let candPoseSets = staticObjects.map(refObj=>{

                let refObjInW = w.annotation.boxes.find(b=>b.obj_track_id == refObj.obj_track_id);
                if (!refObjInW){
                    // not found refobj in this world, give up
                    return null;
                }

                let relativePos = refObj.relativePos;
                let relativeRot = refObj.relativeRot;

                let coord = euler_angle_to_rotate_matrix_3by3(refObjInW.rotation);

                let rp = matmul2(coord, relativePos, 3);
                let newObjPos = {
                    x: refObjInW.position.x + rp[0],
                    y: refObjInW.position.y + rp[1],
                    z: refObjInW.position.z + rp[2],
                };

                let newObjRot = {
                    x: normalizeAngle(refObjInW.rotation.x + relativeRot.x),
                    y: normalizeAngle(refObjInW.rotation.y + relativeRot.y),
                    z: normalizeAngle(refObjInW.rotation.z + relativeRot.z)
                };

             

                return {
                    refTrackId: refObj.obj_track_id,
                    distance: refObj.distance,
                    weight: Math.exp(-refObj.distance * (refObjInW.annotator?1:0.1)),
                    position: newObjPos,
                    rotation: newObjRot,
                };
            });

            candPoseSets = candPoseSets.filter(p=>!!p);


            if (candPoseSets.length == 0) {
                this.debugPropagation(box.obj_track_id, "skip no reference in frame", {
                    frame: w.frameInfo.frame,
                });
                continue;
            }

            // calculate mean pos/rot
            let denorm = candPoseSets.reduce((a,b)=>a+b.weight, 0);

            let newObjPos = {x:0, y:0, z:0};
            let newObjRot = {x:0, y:0, z:0, cosZ: 0, sinZ:0};
            candPoseSets.forEach(p=>{
                newObjPos.x += p.position.x * p.weight;
                newObjPos.y += p.position.y * p.weight;
                newObjPos.z += p.position.z * p.weight;

                newObjRot.x += p.rotation.x * p.weight;
                newObjRot.y += p.rotation.y * p.weight;
                //newObjRot.z += p.rotation.z * p.weight;
                newObjRot.cosZ +=  Math.cos(p.rotation.z) * p.weight;
                newObjRot.sinZ +=  Math.sin(p.rotation.z) * p.weight;
            });

            newObjPos.x /= denorm;
            newObjPos.y /= denorm;
            newObjPos.z /= denorm;
            newObjRot.x /= denorm;
            newObjRot.y /= denorm;
            newObjRot.cosZ /= denorm;
            newObjRot.sinZ /= denorm;
            newObjRot.z = Math.atan2(newObjRot.sinZ, newObjRot.cosZ);
            this.debugPropagation(box.obj_track_id, "coarse candidate", {
                frame: w.frameInfo.frame,
                candCount: candPoseSets.length,
                candidates: candPoseSets.map(p=>({
                    refTrackId: p.refTrackId,
                    distance: Number(p.distance.toFixed(3)),
                    weight: Number(p.weight.toFixed(4)),
                    position: {
                        x: Number(p.position.x.toFixed(3)),
                        y: Number(p.position.y.toFixed(3)),
                        z: Number(p.position.z.toFixed(3)),
                    },
                    rotationZ: Number(p.rotation.z.toFixed(3)),
                })),
                mergedPosition: {
                    x: Number(newObjPos.x.toFixed(3)),
                    y: Number(newObjPos.y.toFixed(3)),
                    z: Number(newObjPos.z.toFixed(3)),
                },
                mergedRotationZ: Number(newObjRot.z.toFixed(3)),
            });
            

            // ignor distant objects

            if (pointsGlobalConfig.ignoreDistantObject){
                let objDistance = Math.sqrt(newObjPos.x * newObjPos.x + newObjPos.y * newObjPos.y + newObjPos.z * newObjPos.z);

                if ((box.scale.z < 2 && objDistance > 100) || objDistance > 150)
                {
                    continue;
                }
            }

            // apply
            let targetBox = existedBox;
            if (existedBox){
                existedBox.position.x = newObjPos.x;
                existedBox.position.y = newObjPos.y;
                existedBox.position.z = newObjPos.z;

                existedBox.rotation.x = newObjRot.x;
                existedBox.rotation.y = newObjRot.y;
                existedBox.rotation.z = newObjRot.z;

                existedBox.scale.x = box.scale.x;
                existedBox.scale.y = box.scale.y;
                existedBox.scale.z = box.scale.z;

                existedBox.annotator="S";
               

                logger.log(`modified box in ${w}`);
            } else{
                let newBox  = w.annotation.add_box(newObjPos, 
                    box.scale, 
                    newObjRot, 
                    box.obj_type, 
                    box.obj_track_id,
                    box.obj_attr);
                newBox.annotator="S";
                
                w.annotation.load_box(newBox);
                targetBox = newBox;
                logger.log(`inserted box in ${w}`);
            }

            let expectedState = this.getExpectedContinuityState(box.obj_track_id, w, worldList);
            let coarseState = this.cloneBoxState(targetBox);
            let refineResult = await this.refinePropagatedBox(targetBox);
            let refineRestore = this.shouldRestoreCoarseAfterRefine(targetBox, coarseState, refineResult);
            this.debugPropagation(box.obj_track_id, "post refine", {
                frame: w.frameInfo.frame,
                direction,
                expectedState: this.summarizeWorldState(expectedState),
                coarseState: this.summarizeBoxState(coarseState),
                refinedState: this.summarizeBoxState(targetBox),
                refineResult,
                refineRestore,
            });
            if (refineRestore.restore){
                this.applyBoxState(targetBox, coarseState);
                console.log(`restore coarse propagated box ${targetBox.obj_track_id} in ${w.frameInfo.frame}: ${refineRestore.reason}`);
            }

            let continuityResult = this.applyContinuityConstraint(targetBox, box.obj_track_id, worldList, expectedState);
            this.debugPropagation(box.obj_track_id, "post continuity", {
                frame: w.frameInfo.frame,
                direction,
                expectedState: this.summarizeWorldState(continuityResult.expectedState),
                boxState: this.summarizeBoxState(targetBox),
            });
            if (targetIsStatic){
                let correction = this.correctStaticPropagationDrift(targetBox, anchorWorldState, box.world);
                this.debugPropagation(box.obj_track_id, "post static correction", {
                    frame: w.frameInfo.frame,
                    direction,
                    correction,
                    boxState: this.summarizeBoxState(targetBox),
                });
                if (correction && correction.discard){
                    this.discardPropagatedBox(targetBox);
                    w.annotation.setModified();
                    continue;
                }
            }
            let filterResult = this.shouldKeepStaticPropagationBox(targetBox);
            filterResult.frame = w.frameInfo.frame;
            filterResult.direction = direction;
            filterResult.supportCount = continuityResult.expectedState ? continuityResult.expectedState.supportCount : 0;
            filterResult.coarsePointCount = refineResult.coarsePointCount;
            filterResult.refinedPointCount = refineResult.refinedPointCount;
            filterResult.finalPointCount = filterResult.pointCount;
            this.debugPropagation(box.obj_track_id, "filter decision", this.summarizeFilterMetrics(filterResult));
            if (!filterResult.keep){
                if (direction !== 0 && this.shouldStopPropagationDirection(refineResult.coarsePointCount, continuityResult.expectedState)){
                    stoppedDirections[String(direction)] = true;
                    this.debugPropagation(box.obj_track_id, "stop direction", {
                        frame: w.frameInfo.frame,
                        direction,
                        reason: filterResult.reason,
                        coarsePointCount: refineResult.coarsePointCount,
                        finalPointCount: filterResult.finalPointCount,
                    });
                }
                this.debugPropagation(box.obj_track_id, "discard sparse propagated box", {
                    frame: w.frameInfo.frame,
                    direction,
                    filter: this.summarizeFilterMetrics(filterResult),
                    boxState: this.summarizeBoxState(targetBox),
                });
                this.discardPropagatedBox(targetBox);
                w.annotation.setModified();
                continue;
            }
            this.debugPropagation(box.obj_track_id, "keep propagated box", {
                frame: w.frameInfo.frame,
                direction,
                filter: this.summarizeFilterMetrics(filterResult),
                boxState: this.summarizeBoxState(targetBox),
            });
            console.log("added box in ", w.frameInfo.frame);
            //saveList.push(w);
            w.annotation.setModified();

        }

    };

    this.followsRef = async function(box){
        //find ref object in current frame
        let world = box.world;
        let refObj = world.annotation.boxes.find(b=>b.obj_track_id == this.marked_object.ann.obj_id);
        if (refObj){
            console.log("found ref obj in current frame");
            world.annotation.setModified()
            
            //compute relative position
            // represent obj in coordinate system of refobj
            
            let coord = euler_angle_to_rotate_matrix_3by3(refObj.rotation);
            let trans = transpose(coord, 3);
            let p = [box.position.x - refObj.position.x, 
                     box.position.y - refObj.position.y, 
                     box.position.z - refObj.position.z];
            const relativePos = matmul2(trans, p, 3);
            const relativeRot = {
                x: box.rotation.x - refObj.rotation.x,
                y: box.rotation.y - refObj.rotation.y,
                z: box.rotation.z - refObj.rotation.z,
            };
            
            let worldList = box.world.data.worldList;
            //let saveList = [];
            for (const w of worldList){
                if (w === box.world){
                    //current frame
                    continue;
                }
                
                let existedBox = w.annotation.boxes.find(b=>b.obj_track_id == box.obj_track_id);
                
                if (existedBox && !existedBox.annotator)
                {
                    // have same objects annotated.
                    // if its generated by machine, lets overwrite it
                    continue;
                }

                let refObjInW = w.annotation.boxes.find(b=>b.obj_track_id == refObj.obj_track_id);
                if (!refObjInW){
                    // not found refobj in this world, give up
                    continue;
                }

                let coord = euler_angle_to_rotate_matrix_3by3(refObjInW.rotation);

                let rp = matmul2(coord, relativePos, 3);
                let newObjPos = {
                    x: refObjInW.position.x + rp[0],
                    y: refObjInW.position.y + rp[1],
                    z: refObjInW.position.z + rp[2],
                };

                let newObjRot = {
                    x: refObjInW.rotation.x + relativeRot.x,
                    y: refObjInW.rotation.y + relativeRot.y,
                    z: refObjInW.rotation.z + relativeRot.z
                };
                
                let targetBox = existedBox;
                if (existedBox){
                    existedBox.position.x = newObjPos.x;
                    existedBox.position.y = newObjPos.y;
                    existedBox.position.z = newObjPos.z;

                    existedBox.rotation.x = newObjRot.x;
                    existedBox.rotation.y = newObjRot.y;
                    existedBox.rotation.z = newObjRot.z;

                    existedBox.scale.x = box.scale.x;
                    existedBox.scale.y = box.scale.y;
                    existedBox.scale.z = box.scale.z;

                    existedBox.annotator="F";
                    existedBox.follows = {
                        obj_track_id: refObj.obj_track_id,
                        relative_position: {
                            x: relativePos[0],
                            y: relativePos[1],
                            z: relativePos[2],
                        },
                        relative_rotation: relativeRot,
                    };

                    logger.log(`modified box in ${w}`);
                } else{
                    let newBox  = w.annotation.add_box(newObjPos, 
                        box.scale, 
                        newObjRot, 
                        box.obj_type, 
                        box.obj_track_id,
                        box.obj_attr);
                    newBox.annotator="F";
                    newBox.follows = {
                        obj_track_id: refObj.obj_track_id,
                        relative_position: {
                            x: relativePos[0],
                            y: relativePos[1],
                            z: relativePos[2],
                        },
                        relative_rotation: relativeRot,
                    };

                    w.annotation.load_box(newBox);
                    targetBox = newBox;
                    logger.log(`inserted box in ${w}`);
                }

                let coarseState = this.cloneBoxState(targetBox);
                let refineResult = await this.refinePropagatedBox(targetBox);
                let refineRestore = this.shouldRestoreCoarseAfterRefine(targetBox, coarseState, refineResult);
                if (refineRestore.restore){
                    this.applyBoxState(targetBox, coarseState);
                    console.log(`restore coarse follow-ref box ${targetBox.obj_track_id} in ${w.frameInfo.frame}: ${refineRestore.reason}`);
                }
                console.log("added box in ", w.frameInfo.frame);
                //saveList.push(w);
                w.annotation.setModified();
            }

            //saveWorldList(saveList);
        }
    };

    this.syncFollowers = function(box){
        let world = box.world;
        let allFollowers = world.annotation.boxes.filter(b=>b.follows && b.follows.obj_track_id === box.obj_track_id);

        if (allFollowers.length == 0){
            console.log("no followers");
            return;
        }

        let refObj = box;
        let coord = euler_angle_to_rotate_matrix_3by3(refObj.rotation);
        

        allFollowers.forEach(fb=>{
            let relpos = [fb.follows.relative_position.x,
                fb.follows.relative_position.y,
                fb.follows.relative_position.z,
            ];

            let rp = matmul2(coord, relpos, 3);
            
            fb.position.x = refObj.position.x + rp[0];
            fb.position.y = refObj.position.y + rp[1];
            fb.position.z = refObj.position.z + rp[2];

            fb.rotation.x = refObj.rotation.x + fb.follows.relative_rotation.x;
            fb.rotation.y = refObj.rotation.y + fb.follows.relative_rotation.y;
            fb.rotation.z = refObj.rotation.z + fb.follows.relative_rotation.z;
        });
    };

    this.paste_bbox=function(pos, add_box){
    
        if (!pos)
           pos = this.marked_object.ann.psr.position;
        else
           pos.z = this.marked_object.ann.psr.position.z;
    
        return  add_box(pos, this.marked_object.ann.psr.scale, this.marked_object.ann.psr.rotation,
            this.marked_object.ann.obj_type, this.marked_object.ann.obj_id, this.marked_object.ann.obj_attr);    
    };
    
    
    // this.auto_adjust_bbox=function(box, done, on_box_changed){
    
    //     saveWorld(function(){
    //         do_adjust(box, on_box_changed);
    //     });
    //     let _self =this;
    //     function do_adjust(box, on_box_changed){
    //         console.log("auto adjust highlighted bbox");
    
    //         var xhr = new XMLHttpRequest();
    //         // we defined the xhr
            
    //         xhr.onreadystatechange = function () {
    //             if (this.readyState != 4) return;
            
    //             if (this.status == 200) {
    //                 console.log(this.responseText)
    //                 console.log(box.position);
    //                 console.log(box.rotation);
    
    
    //                 var trans_mat = JSON.parse(this.responseText);
    
    //                 var rotation = Math.atan2(trans_mat[4], trans_mat[0]) + box.rotation.z;
    //                 var transform = {
    //                     x: -trans_mat[3],
    //                     y: -trans_mat[7],
    //                     z: -trans_mat[11],
    //                 }
    
                    
                    
    //                 /*
    //                 cos  sin    x 
    //                 -sin cos    y 
    //                 */
    //                 var new_pos = {
    //                     x: Math.cos(-rotation) * transform.x + Math.sin(-rotation) * transform.y,
    //                     y: -Math.sin(-rotation) * transform.x + Math.cos(-rotation) * transform.y,
    //                     z: transform.z,
    //                 };
    
    
    //                 box.position.x += new_pos.x;
    //                 box.position.y += new_pos.y;
    //                 box.position.z += new_pos.z;
                    
                    
    
    //                 box.scale.x = marked_object.scale.x;
    //                 box.scale.y = marked_object.scale.y;
    //                 box.scale.z = marked_object.scale.z;
    
    //                 box.rotation.z -= Math.atan2(trans_mat[4], trans_mat[0]);
    
    //                 console.log(box.position);
    //                 console.log(box.rotation);
    
    //                 on_box_changed(box);
            
    //                 _self.header.mark_changed_flag();
    
    //                 if (done){
    //                     done();
    //                 }
    //             }
            
    //             // end of state change: it can be after some time (async)
    //         };
            
    //         xhr.open('GET', 
    //                 "/auto_adjust"+"?scene="+marked_object.scene + "&"+
    //                             "ref_frame=" + marked_object.frame + "&" +
    //                             "object_id=" + marked_object.obj_track_id + "&" +                           
    //                             "adj_frame=" + data.world.frameInfo.frame, 
    //                 true);
    //         xhr.send();
    //     }
    // };

    this.smart_paste=function(selected_box, add_box, on_box_changed){
        var box = selected_box;
        if (!box){
            let sceneP =  this.mouse.get_mouse_location_in_world()
            // trans pos to world local pos
            //let pos = this.data.world.scenePosToLidar(sceneP);
            box = this.paste_bbox(pos, add_box);
        }
        else if (this.marked_object){
            box.scale.x = this.marked_object.ann.psr.scale.x;
            box.scale.y = this.marked_object.ann.psr.scale.y;
            box.scale.z = this.marked_object.ann.psr.scale.z;
        }
        
        // this.auto_adjust_bbox(box,
        //         function(){saveWorld();},
        //         on_box_changed);
    
        // this.header.mark_changed_flag();

        
        
        this.boxOp.auto_rotate_xyz(box, null, null, 
            on_box_changed,
            "noscaling");
    };
    
}


export {AutoAdjust}
