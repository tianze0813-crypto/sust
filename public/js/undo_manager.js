import * as THREE from './lib/three.module.js';

const HISTORY_MAX_DEPTH = 50;
const UNDO_MERGE_INTERVAL_MS = 500;

class UndoManager {
    constructor() {
        this.historyStack = [];
        this.redoStack = [];
        this.lastSnapshotTime = 0;
        this.enabled = true;
    }

    takeSnapshot(annotation) {
        if (!this.enabled || !annotation || !annotation.boxes) {
            return;
        }

        const now = Date.now();
        const snapshot = this._serializeBoxes(annotation.boxes);

        if (this.historyStack.length > 0 && (now - this.lastSnapshotTime) < UNDO_MERGE_INTERVAL_MS) {
            this.historyStack[this.historyStack.length - 1] = snapshot;
        } else {
            this.historyStack.push(snapshot);
            if (this.historyStack.length > HISTORY_MAX_DEPTH) {
                this.historyStack.shift();
            }
        }

        this.redoStack = [];
        this.lastSnapshotTime = now;
    }

    undo(annotation) {
        if (!this.enabled || this.historyStack.length === 0) {
            return false;
        }

        const currentSnapshot = this._serializeBoxes(annotation.boxes);
        this.redoStack.push(currentSnapshot);

        const previousSnapshot = this.historyStack.pop();
        this._restoreBoxes(annotation, previousSnapshot);

        return true;
    }

    redo(annotation) {
        if (!this.enabled || this.redoStack.length === 0) {
            return false;
        }

        const currentSnapshot = this._serializeBoxes(annotation.boxes);
        this.historyStack.push(currentSnapshot);

        const nextSnapshot = this.redoStack.pop();
        this._restoreBoxes(annotation, nextSnapshot);

        return true;
    }

    clear() {
        this.historyStack = [];
        this.redoStack = [];
        this.lastSnapshotTime = 0;
    }

    _serializeBoxes(boxes) {
        return boxes.map((box) => ({
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
            obj_type: box.obj_type,
            obj_id: box.obj_id,
            obj_track_id: box.obj_track_id,
            obj_attr: box.obj_attr,
            obj_local_id: box.obj_local_id,
            annotator: box.annotator,
            follows: box.follows,
        }));
    }

    _restoreBoxes(annotation, snapshot) {
        const world = annotation.world;
        const webglGroup = annotation.webglGroup;

        annotation.boxes.forEach((box) => {
            if (box.boxEditor) {
                box.boxEditor.detach("donthide");
            }
            webglGroup.remove(box);
            box.geometry.dispose();
            box.material.dispose();
        });

        annotation.boxes = [];

        snapshot.forEach((data) => {
            const pos = new THREE.Vector3(data.position.x, data.position.y, data.position.z);
            const scale = new THREE.Vector3(data.scale.x, data.scale.y, data.scale.z);
            const rotation = new THREE.Vector3(data.rotation.x, data.rotation.y, data.rotation.z);

            const box = annotation.createCuboid(pos, scale, rotation, data.obj_type, data.obj_track_id, data.obj_attr);
            box.obj_id = data.obj_id;
            box.obj_local_id = data.obj_local_id;
            box.annotator = data.annotator;
            box.follows = data.follows;

            annotation.boxes.push(box);
            webglGroup.add(box);
        });

        annotation.sort_boxes();
        annotation.color_boxes();
        annotation.setModified();

        if (world.lidar) {
            world.lidar.recolor_all_points();
        }
    }
}

export { UndoManager }
