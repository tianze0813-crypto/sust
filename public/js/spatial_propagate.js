import * as THREE from './lib/three.module.js';

class SpatialPropagation {
    constructor(editor) {
        this.editor = editor;
        this.state = null;
        this.isRunning = false;
        this.direction = null;
        this.currentStepIdx = -1;
        this.previousCount = null;
        this.previousPos = null;
        this.previousYaw = null;
        this.currentSourceFrame = null;
        this.currentSourcePsr = null;
        this.fixedRefFrame = null;
        this.fixedRefPsr = null;
        this.stackFrameRadius = 4;
        this.applyFrameIds = null;
        this.fixedRefIdx = -1;
        this.minApplyIdx = -1;
        this.maxApplyIdx = -1;
        this.progressBaseIdx = 0;
        this.progressTotal = 0;
        this.propagatedBoxes = [];
        this.onStopCallback = null;
        this.sourceMode = "fixed-reference";
    }

    _round(value) {
        if (value === null || value === undefined || Number.isNaN(Number(value))) {
            return null;
        }
        return Number(Number(value).toFixed(3));
    }

    _formatPosition(position) {
        if (!position) {
            return null;
        }
        return {
            x: this._round(position.x),
            y: this._round(position.y),
            z: this._round(position.z),
        };
    }

    _formatRotation(rotation) {
        if (!rotation) {
            return null;
        }
        return {
            x: this._round(rotation.x),
            y: this._round(rotation.y),
            z: this._round(rotation.z),
        };
    }

    _formatScale(scale) {
        if (!scale) {
            return null;
        }
        return {
            x: this._round(scale.x),
            y: this._round(scale.y),
            z: this._round(scale.z),
        };
    }

    _formatPsr(psr) {
        if (!psr) {
            return null;
        }
        return {
            position: this._formatPosition(psr.position),
            rotation: this._formatRotation(psr.rotation),
            scale: this._formatScale(psr.scale),
        };
    }

    _clonePsr(psr) {
        if (!psr) {
            return null;
        }
        return JSON.parse(JSON.stringify(psr));
    }

    _positionDelta(fromPos, toPos) {
        if (!fromPos || !toPos) {
            return null;
        }
        return {
            dx: this._round((toPos.x || 0) - (fromPos.x || 0)),
            dy: this._round((toPos.y || 0) - (fromPos.y || 0)),
            dz: this._round((toPos.z || 0) - (fromPos.z || 0)),
        };
    }

    _xyDistance(fromPos, toPos) {
        if (!fromPos || !toPos) {
            return null;
        }
        const dx = (toPos.x || 0) - (fromPos.x || 0);
        const dy = (toPos.y || 0) - (fromPos.y || 0);
        return Math.sqrt(dx * dx + dy * dy);
    }

    _getAutoFitAcceptThreshold(psr) {
        const scale = psr && psr.scale ? psr.scale : null;
        const minPlanarSize = scale ? Math.min(Math.abs(scale.x || 0), Math.abs(scale.y || 0)) : 0;
        const dynamicThreshold = minPlanarSize > 0 ? (minPlanarSize * 0.2) : 0;
        return Math.max(0.25, Math.min(0.45, dynamicThreshold || 0.35));
    }

    _getIndependentSourceFrame() {
        return this.fixedRefFrame || (this.state ? this.state.anchorFrame : null);
    }

    _getIndependentSourcePsr() {
        if (this.fixedRefPsr) {
            return this.fixedRefPsr;
        }
        return this.state ? this.state.boxPsr : null;
    }

    _debug(label, payload = null) {
        if (payload !== null && payload !== undefined) {
            let serialized = "";
            try {
                serialized = " " + JSON.stringify(payload);
            } catch (_err) {
                serialized = " [unserializable]";
            }
            console.log(`[StackAnno] ${label}${serialized}`);
        } else {
            console.log(`[StackAnno] ${label}`);
        }
    }

    init(scene, anchorFrame, objId, boxPsr, referenceCount, frameIds, currentIdx, objType, objAttr, fixedRefFrame=null, fixedRefPsr=null, stackFrameRadius=4, applyFrameIds=null) {
        this.state = {
            scene: scene,
            anchorFrame: anchorFrame,
            objId: objId,
            boxPsr: boxPsr,
            referenceCount: referenceCount,
            frameIds: frameIds,
            anchorIdx: currentIdx,
            objType: objType || "",
            objAttr: objAttr || "",
        };
        this.fixedRefFrame = fixedRefFrame || anchorFrame;
        this.fixedRefPsr = fixedRefPsr ? JSON.parse(JSON.stringify(fixedRefPsr)) : JSON.parse(JSON.stringify(boxPsr));
        this.stackFrameRadius = stackFrameRadius || 4;
        this.applyFrameIds = Array.isArray(applyFrameIds) ? applyFrameIds.map(x => parseInt(x, 10)).filter(x => !Number.isNaN(x)) : null;
        this.fixedRefIdx = this.state.frameIds.indexOf(parseInt(this.fixedRefFrame, 10));
        this._updateApplyWindow();
        this.propagatedBoxes = [];
        this.isRunning = false;
        this.direction = null;
        this.currentStepIdx = -1;
        this.progressBaseIdx = currentIdx;
        this.progressTotal = 0;
        this.previousCount = null;
        this.previousPos = null;
        this.previousYaw = null;
        this.currentSourceFrame = this._getIndependentSourceFrame();
        this.currentSourcePsr = this._clonePsr(this._getIndependentSourcePsr());
    }

    async start(direction, onStep, onStop) {
        if (!this.state) {
            console.error("propagation not initialized");
            return;
        }

        this.isRunning = true;
        this.direction = direction;
        this.onStopCallback = onStop;
        this._updateApplyWindow();

        if (direction === "forward") {
            this.currentStepIdx = this.state.anchorIdx + 1;
            const startIdx = Math.max(this.state.anchorIdx + 1, this.minApplyIdx);
            const endIdx = Math.min(this.maxApplyIdx, this.state.frameIds.length - 1);
            this.progressBaseIdx = startIdx;
            this.progressTotal = endIdx >= startIdx ? (endIdx - startIdx + 1) : 0;
        } else if (direction === "backward") {
            this.currentStepIdx = this.state.anchorIdx - 1;
            const startIdx = Math.min(this.state.anchorIdx - 1, this.maxApplyIdx);
            const endIdx = Math.max(this.minApplyIdx, 0);
            this.progressBaseIdx = startIdx;
            this.progressTotal = startIdx >= endIdx ? (startIdx - endIdx + 1) : 0;
        } else {
            console.error("invalid direction:", direction);
            return;
        }

        if (this.progressTotal <= 0) {
            this.stop();
            return;
        }

        this.previousCount = this.state.referenceCount;
        this.previousPos = this.state.boxPsr.position;
        this.previousYaw = this.state.boxPsr && this.state.boxPsr.rotation ? this.state.boxPsr.rotation.z : null;
        this.currentSourceFrame = this._getIndependentSourceFrame();
        this.currentSourcePsr = this._clonePsr(this._getIndependentSourcePsr());

        this._debug("start", {
            scene: this.state.scene,
            objId: this.state.objId,
            direction: direction,
            anchorFrame: this.state.anchorFrame,
            fixedRefFrame: this.fixedRefFrame,
            stackFrameRadius: this.stackFrameRadius,
            applyWindow: {
                minIdx: this.minApplyIdx,
                maxIdx: this.maxApplyIdx,
                total: this.progressTotal,
            },
            fixedRefPsr: this._formatPsr(this.fixedRefPsr),
            sourcePsr: this._formatPsr(this.currentSourcePsr),
            sourceMode: this.sourceMode,
        });

        await this._propagateLoop(onStep);
    }

    async _propagateLoop(onStep) {
        while (this.isRunning) {
            if (this.currentStepIdx < 0 || this.currentStepIdx >= this.state.frameIds.length) {
                this.stop();
                break;
            }
            if (this.currentStepIdx < this.minApplyIdx || this.currentStepIdx > this.maxApplyIdx) {
                this.stop();
                break;
            }

            const tgtFrame = this.state.frameIds[this.currentStepIdx];

            try {
                const result = await this._propagateStep(tgtFrame);

                if (result.error) {
                    console.error("propagation error:", result.error);
                    this.stop();
                    break;
                }

                const stepResult = {
                    frame: tgtFrame,
                    psr: result.psr,
                    pointCount: result.point_count,
                    method: result.method,
                    score: result.score,
                    direction: this.direction,
                };

                let applyResult = null;
                if (onStep) {
                    applyResult = await onStep(stepResult);
                }

                let confirmedPsr = applyResult && applyResult.finalPsr ? applyResult.finalPsr : result.psr;
                let confirmedCount = result.point_count;
                this._debug("step-confirmed", {
                    frame: tgtFrame,
                    objId: this.state.objId,
                    backendPsr: this._formatPsr(result.psr),
                    confirmedPsr: this._formatPsr(confirmedPsr),
                    pointCount: confirmedCount,
                    sourceMode: this.sourceMode,
                });
                this.currentSourceFrame = this._getIndependentSourceFrame();
                this.currentSourcePsr = this._clonePsr(this._getIndependentSourcePsr());
                this.previousCount = confirmedCount;
                this.previousPos = result.psr ? this._clonePsr(result.psr.position) : null;
                this.previousYaw = confirmedPsr && confirmedPsr.rotation ? confirmedPsr.rotation.z : null;
                this._advanceStep();

            } catch (err) {
                console.error("propagation step failed:", err);
                this.stop();
                break;
            }
        }
    }

    async _propagateStep(tgtFrame) {
        const s = this.state;
        const sourceFrame = this._getIndependentSourceFrame() || s.anchorFrame;
        const sourcePsr = this._clonePsr(this._getIndependentSourcePsr() || s.boxPsr);
        this._debug("step-request", {
            frame: tgtFrame,
            objId: s.objId,
            sourceFrame: sourceFrame,
            fixedRefFrame: this.fixedRefFrame || s.anchorFrame,
            sourcePsr: this._formatPsr(sourcePsr),
            sourceMode: this.sourceMode,
        });
        const params = new URLSearchParams({
            scene: s.scene,
            anchor_frame: s.anchorFrame,
            obj_id: s.objId,
            tgt_frame: tgtFrame,
            reference_count: s.referenceCount || 0,
            previous_count: this.previousCount || 0,
            previous_pos: JSON.stringify(this.previousPos || {}),
            previous_yaw: this.previousYaw !== null && this.previousYaw !== undefined ? String(this.previousYaw) : "",
            src_frame: sourceFrame,
            src_psr: JSON.stringify(sourcePsr),
            fixed_ref_frame: this.fixedRefFrame || s.anchorFrame,
            fixed_ref_psr: JSON.stringify(this.fixedRefPsr || s.boxPsr),
            stack_frame_radius: this.stackFrameRadius || 4,
        });

        const resp = await fetch("/propagate_check?" + params.toString());
        const data = await resp.json();
        this._debug("step-response", {
            frame: tgtFrame,
            objId: s.objId,
            method: data.method || null,
            pointCount: data.point_count,
            score: this._round(data.score),
            shapeScore: this._round(data.shape_score),
            smoothBonus: this._round(data.smooth_bonus),
            yawDelta: this._round(data.yaw_delta),
            backendPsr: this._formatPsr(data.psr),
            error: data.error || null,
        });
        return data;
    }

    setFixedReference(frame, psr, stackFrameRadius, applyFrameIds=null) {
        this.fixedRefFrame = frame || (this.state ? this.state.anchorFrame : null);
        this.fixedRefPsr = psr ? JSON.parse(JSON.stringify(psr)) : null;
        if (stackFrameRadius !== undefined && stackFrameRadius !== null) {
            this.stackFrameRadius = stackFrameRadius;
        }
        if (Array.isArray(applyFrameIds)) {
            this.applyFrameIds = applyFrameIds.map(x => parseInt(x, 10)).filter(x => !Number.isNaN(x));
        } else if (applyFrameIds === null) {
            this.applyFrameIds = null;
        }
        if (this.state && this.state.frameIds) {
            this.fixedRefIdx = this.state.frameIds.indexOf(parseInt(this.fixedRefFrame, 10));
            this._updateApplyWindow();
        }
    }

    _updateApplyWindow() {
        if (!this.state || !this.state.frameIds || !this.state.frameIds.length) {
            this.minApplyIdx = -1;
            this.maxApplyIdx = -1;
            return;
        }
        const refIdx = this.fixedRefIdx >= 0 ? this.fixedRefIdx : this.state.anchorIdx;
        if (this.applyFrameIds && this.applyFrameIds.length > 0) {
            const indices = this.applyFrameIds
                .map(frameId => this.state.frameIds.indexOf(frameId))
                .filter(idx => idx >= 0);
            if (indices.length > 0) {
                this.minApplyIdx = Math.min(...indices);
                this.maxApplyIdx = Math.max(...indices);
                return;
            }
        }

        const radius = Math.max(0, parseInt(this.stackFrameRadius || 0, 10));
        this.minApplyIdx = Math.max(0, refIdx - radius);
        this.maxApplyIdx = Math.min(this.state.frameIds.length - 1, refIdx + radius);
    }

    _getCurrentBoxPsr() {
        const world = this.editor.data.world;
        if (!world || !world.annotation) {
            return null;
        }

        const box = world.annotation.findBoxByTrackId(this.state.objId);
        if (!box) {
            return null;
        }

        return {
            position: {
                x: box.position.x,
                y: box.position.y,
                z: box.position.z,
            },
            scale: {
                x: box.scale.x,
                y: box.scale.y,
                z: box.scale.z,
            },
            rotation: {
                x: box.rotation.x,
                y: box.rotation.y,
                z: box.rotation.z,
            },
        };
    }

    _advanceStep() {
        if (this.direction === "forward") {
            this.currentStepIdx++;
        } else {
            this.currentStepIdx--;
        }
    }

    stop() {
        this.isRunning = false;
        if (this.onStopCallback) {
            this.onStopCallback();
        }
    }

    async applyBoxToCurrentFrame(psr, objType, objAttr) {
        const world = this.editor.data.world;
        if (!world) return null;

        const annotation = world.annotation;
        const pos = new THREE.Vector3(psr.position.x, psr.position.y, psr.position.z);
        const scale = new THREE.Vector3(psr.scale.x, psr.scale.y, psr.scale.z);

        const existingBox = annotation.findBoxByTrackId(this.state.objId);
        if (existingBox) {
            const beforeApply = this._getCurrentBoxPsr();
            this._debug("frame-apply-begin", {
                frame: world.frameInfo ? world.frameInfo.frame : null,
                objId: this.state.objId,
                existing: true,
                backendPsr: this._formatPsr(psr),
                boxBeforeApply: this._formatPsr(beforeApply),
                applyDelta: this._positionDelta(beforeApply ? beforeApply.position : null, psr.position),
            });
            existingBox.position.copy(pos);
            existingBox.scale.copy(scale);
            existingBox.rotation.set(psr.rotation.x, psr.rotation.y, psr.rotation.z);
            existingBox.obj_type = objType || existingBox.obj_type;
            existingBox.obj_attr = objAttr || existingBox.obj_attr;
            existingBox.changed = true;
            annotation.setModified();
            const finalPsr = await this._refineWithSustAutoFit(existingBox);
            this._debug("frame-apply-done", {
                frame: world.frameInfo ? world.frameInfo.frame : null,
                objId: this.state.objId,
                finalPsr: this._formatPsr(finalPsr),
                fitDeltaFromBackend: this._positionDelta(psr.position, finalPsr ? finalPsr.position : null),
                fitDeltaFromBefore: this._positionDelta(beforeApply ? beforeApply.position : null, finalPsr ? finalPsr.position : null),
            });
            this.editor.render();
            return { box: existingBox, finalPsr: finalPsr || this._clonePsr(psr) };
        }

        this._debug("frame-apply-begin", {
            frame: world.frameInfo ? world.frameInfo.frame : null,
            objId: this.state.objId,
            existing: false,
            backendPsr: this._formatPsr(psr),
        });
        const box = annotation.add_box(pos, scale, new THREE.Vector3(psr.rotation.x, psr.rotation.y, psr.rotation.z), objType, this.state.objId, objAttr);
        box.changed = true;
        annotation.setModified();
        const finalPsr = await this._refineWithSustAutoFit(box);
        this._debug("frame-apply-done", {
            frame: world.frameInfo ? world.frameInfo.frame : null,
            objId: this.state.objId,
            finalPsr: this._formatPsr(finalPsr),
            fitDeltaFromBackend: this._positionDelta(psr.position, finalPsr ? finalPsr.position : null),
        });
        this.editor.render();
        return { box: box, finalPsr: finalPsr || this._clonePsr(psr) };
    }

    async _refineWithSustAutoFit(box) {
        if (!box || !this.editor || !this.editor.boxOp || !box.world || !box.world.lidar) {
            this._debug("auto-fit-skipped", {
                frame: box && box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
                objId: this.state ? this.state.objId : null,
            });
            return box;
        }

        const beforeFit = this._getCurrentBoxPsr();
        this._debug("auto-fit-begin", {
            frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
            objId: this.state.objId,
            beforeFit: this._formatPsr(beforeFit),
        });

        await this.editor.boxOp.auto_rotate_xyz(
            box,
            null,
            null,
            (b) => this.editor.on_box_changed(b),
            true,
            null
        );
        const afterFit = this._getCurrentBoxPsr();
        let finalPsr = afterFit ? this._clonePsr(afterFit) : this._getCurrentBoxPsr();

        if (beforeFit && finalPsr && finalPsr.position) {
            const lockedZ = beforeFit.position ? beforeFit.position.z : null;
            const movedZ = finalPsr.position.z;
            if (lockedZ !== null && lockedZ !== undefined && movedZ !== lockedZ) {
                box.position.z = lockedZ;
                box.changed = true;
                if (box.world && box.world.annotation) {
                    box.world.annotation.setModified();
                }
                this.editor.on_box_changed(box);
                finalPsr = this._getCurrentBoxPsr();
                this._debug("auto-fit-lock-z", {
                    frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
                    objId: this.state.objId,
                    lockedZ: this._round(lockedZ),
                    rawZ: this._round(movedZ),
                    restoredZ: finalPsr && finalPsr.position ? this._round(finalPsr.position.z) : null,
                });
            }
        }

        const xyDistance = this._xyDistance(beforeFit ? beforeFit.position : null, finalPsr ? finalPsr.position : null);
        const acceptThreshold = this._getAutoFitAcceptThreshold(beforeFit);
        if (beforeFit && finalPsr && finalPsr.position && xyDistance !== null && xyDistance > acceptThreshold) {
            const rejectedPsr = this._clonePsr(finalPsr);
            box.position.x = beforeFit.position.x;
            box.position.y = beforeFit.position.y;
            if (beforeFit.position && beforeFit.position.z !== undefined && beforeFit.position.z !== null) {
                box.position.z = beforeFit.position.z;
            }
            box.rotation.set(beforeFit.rotation.x, beforeFit.rotation.y, beforeFit.rotation.z);
            box.scale.set(beforeFit.scale.x, beforeFit.scale.y, beforeFit.scale.z);
            box.changed = true;
            if (box.world && box.world.annotation) {
                box.world.annotation.setModified();
            }
            this.editor.on_box_changed(box);
            finalPsr = this._getCurrentBoxPsr();
            this._debug("auto-fit-rejected", {
                frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
                objId: this.state.objId,
                threshold: this._round(acceptThreshold),
                planarMove: this._round(xyDistance),
                backendPsr: this._formatPsr(beforeFit),
                rejectedPsr: this._formatPsr(rejectedPsr),
                acceptedPsr: this._formatPsr(finalPsr),
            });
        }

        this._debug("auto-fit-end", {
            frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
            objId: this.state.objId,
            afterFit: this._formatPsr(finalPsr),
            fitDelta: this._positionDelta(beforeFit ? beforeFit.position : null, finalPsr ? finalPsr.position : null),
            yawDelta: (beforeFit && finalPsr && beforeFit.rotation && finalPsr.rotation) ?
                this._round((finalPsr.rotation.z || 0) - (beforeFit.rotation.z || 0)) :
                null,
        });
        return finalPsr;
    }

    async navigateToFrame(frame) {
        const scene = this.state.scene;
        return new Promise((resolve) => {
            this.editor.load_world(scene, String(frame), () => {
                resolve();
            });
        });
    }

    getProgress() {
        if (!this.state) return { current: 0, total: 0 };
        let current = this.currentStepIdx;
        if (this.direction === "forward") {
            current = this.currentStepIdx - this.progressBaseIdx;
        } else if (this.direction === "backward") {
            current = this.progressBaseIdx - this.currentStepIdx;
        }
        current = Math.max(0, Math.min(current, Math.max(0, this.progressTotal - 1)));
        return {
            current: current,
            total: this.progressTotal || 0,
            anchorIdx: this.state.anchorIdx,
        };
    }
}

export { SpatialPropagation }
