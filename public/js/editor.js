import * as THREE from './lib/three.module.js';

import {ViewManager} from "./view.js";
import {FastToolBox, FloatLabelManager} from "./floatlabel.js?v=2";
import {Mouse} from "./mouse.js";
import {BoxEditor, BoxEditorManager} from "./box_editor.js?v=15";
import {ImageContextManager} from "./image.js";
import {globalObjectCategory} from "./obj_cfg.js";

import {objIdManager} from "./obj_id_list.js";
import {Header} from "./header.js";
import {BoxOp} from './box_op.js';
import {AutoAdjust} from "./auto-adjust.js?v=21";
import {PlayControl} from "./play.js";
import {reloadWorldList, saveWorldList, saveWorldListImmediate} from "./save.js";
import {logger, create_logger} from "./log.js";
import {autoAnnotate} from "./auto_annotate.js";
import {Calib} from "./calib.js";
import {Trajectory} from "./trajectory.js";
import { ContextMenu } from './context_menu.js';
import { InfoBox } from './info_box.js';
import {CropScene} from './crop_scene.js';
import { ConfigUi } from './config_ui.js';
import { MovableView } from './popup_dialog.js';
import {globalKeyDownManager} from './keydown_manager.js';
import {vector_range} from "./util.js"
import { checkScene } from './error_check.js';

function Editor(editorUi, wrapperUi, editorCfg, data, name="editor"){

    // create logger before anything else.
    create_logger(editorUi.querySelector("#log-wrapper"), editorUi.querySelector("#log-button"));
    this.logger = logger;

    this.editorCfg = editorCfg;
    this.sideview_enabled = true;
    this.editorUi = editorUi;
    this.wrapperUi = wrapperUi;
    this.container = null;
    this.name = name;

    this.data = data;
    this.scene = null;
    this.renderer = null;
    this.selected_box = null;
    this.stackPreviewMesh = null;
    this.stackReference = null;
    this.batchUndoStack = [];
    this.windowWidth = null;
    this.windowHeight= null;
    this.floatLabelManager = null;
    this.operation_state = {
            key_pressed : false,
            box_navigate_index:0,
        };
    this.view_state = {
        lock_obj_track_id : "",
        lock_obj_in_highlight : false,  // focus mode
        autoLock: function(trackid, focus){
            this.lock_obj_track_id = trackid;
            this.lock_obj_in_highlight = focus;
        }
    };
    this.calib = new Calib(this.data, this);

    this.header = null;
    this.imageContextManager = null;
    this.boxOp = null;
    this.boxEditorManager  = null; 
    this.params={};

    this.currentMainEditor = this;  // who is on focus, this or batch-editor-manager?

    this.init = function(editorUi) {
    
        let self = this;
        this.editorUi = editorUi;
    
        


        this.playControl = new PlayControl(this.data);

        this.configUi = new ConfigUi(editorUi.querySelector("#config-button"), editorUi.querySelector("#config-wrapper"), this);

        this.header = new Header(editorUi.querySelector("#header"), this.data, this.editorCfg,
            (e)=>{
                this.scene_changed(e.currentTarget.value);
                //event.currentTarget.blur();
            },        
            (e)=>{this.frame_changed(e)},
            (e)=>{this.object_changed(e)},
            (e)=>{this.camera_changed(e)}        
        );


        //
        // that way, the operation speed may be better
        // if we load all worlds, we can speed up batch-mode operations, but the singl-world operations slows down.
        // if we use two seperate scenes. can we solve this problem?
        //
        this.scene = new THREE.Scene();
        this.mainScene = this.scene; //new THREE.Scene();
        
        this.data.set_webglScene(this.scene, this.mainScene);

        

        this.renderer = new THREE.WebGLRenderer( { antialias: true, preserveDrawingBuffer: true } );
        this.renderer.setPixelRatio( window.devicePixelRatio );
        
        this.container = editorUi.querySelector("#container");
        this.container.appendChild( this.renderer.domElement );   
        
        

        this.boxOp = new BoxOp(this.data);
        this.viewManager = new ViewManager(this.container, this.scene, this.mainScene, this.renderer, 
            function(){self.render();}, 
            function(box){self.on_box_changed(box)},
            this.editorCfg,
            function(){
                if (self.data.world && self.data.world.annotation) {
                    self.data.world.annotation.undoManager.takeSnapshot(self.data.world.annotation);
                }
            });
        

        this.imageContextManager = new ImageContextManager(
                this.editorUi.querySelector("#content"), 
                this.editorUi.querySelector("#camera-selector"),
                this.editorCfg,
                (lidar_points)=>this.on_img_click(lidar_points));


        if (!this.editorCfg.disableRangeCircle)
            this.addRangeCircle();
    
        this.floatLabelManager = new FloatLabelManager(this.editorUi, this.container, this.viewManager.mainView,function(box){self.selectBox(box);});
        this.fastToolBox = new FastToolBox(this.editorUi.querySelector("#obj-editor"), (event)=>this.handleFastToolEvent(event));
        //this.controlGui = this.init_gui();
        
        this.axis = new THREE.AxesHelper(1);

        this.scene.add(this.axis);
    
        window.addEventListener( 'resize', function(){self.onWindowResize();}, false );
        

        if (!this.editorCfg.disableMainViewKeyDown){
            // this.container.onmouseenter = (event)=>{
            //     this.container.focus();
            // };

            // this.container.onmouseleave = (event)=>{
            //     this.container.blur();                
            // };

            //this.container.addEventListener( 'keydown', function(e){self.keydown(e);} );
            //this.editorUi.addEventListener( 'keydown', e=>this.keydown(e); );

            this.keydownHandler = (event)=>this.keydown(event);
            //this.keydownDisabled = false;
            //document.removeEventListener('keydown', this.keydownHandler);
            //document.addEventListener( 'keydown', this.keydownHandler);
            globalKeyDownManager.register(this.keydownHandler, "main editor");
        }

        this.globalKeyDownManager = globalKeyDownManager;

        this.objectTrackView = new Trajectory(
            this.editorUi.querySelector("#object-track-wrapper")
        );

        this.infoBox = new InfoBox(
            this.editorUi.querySelector("#info-wrapper")
        );

        this.cropScene = new CropScene(
            this.editorUi.querySelector("#crop-scene-wrapper"),
            this
        );

        this.contextMenu = new ContextMenu(this.editorUi.querySelector("#context-menu-wrapper"));        

        

        this.boxEditorManager = new BoxEditorManager(
            document.querySelector("#batch-box-editor"),
            this.viewManager,
            this.objectTrackView,
            this.editorCfg,
            this.boxOp,
            this.header,
            this.contextMenu,
            this.configUi,
            (b)=>this.on_box_changed(b),
            (b,r)=>this.remove_box(b,r),   // on box remove
            ()=>{
                // this.on_load_world_finished(this.data.world);
                // this.imageContextManager.hide();
                // this.floatLabelManager.hide();

                // this.viewManager.mainView.disable();
                // this.boxEditor.hide();
                // this.hideGridLines();
                // this.controlGui.hide();
                
            });  //func_on_annotation_reloaded
        this.boxEditorManager.hide();
         
        let boxEditorUi = this.editorUi.querySelector("#main-box-editor-wrapper");
        this.boxEditor= new BoxEditor(
            boxEditorUi,
            null,  // no box editor manager
            this.viewManager, 
            this.editorCfg, 
            this.boxOp, 
            (b)=>this.on_box_changed(b),
            (b)=>this.remove_box(b),
            "main-boxe-ditor");
        this.boxEditor.detach(); // hide it
        this.boxEditor.setResize("both");
        this.boxEditor.moveHandle = new MovableView(
            boxEditorUi.querySelector("#focuscanvas"),
            boxEditorUi.querySelector("#sub-views"),
            ()=>{
                this.boxEditor.update();
                this.render();
            }
        );

        this.mouse = new Mouse(
            this.viewManager.mainView,
            this.operation_state,
            this.container, 
            this.editorUi,
            function(ev){self.handleLeftClick(ev);}, 
            function(ev){self.handleRightClick(ev);}, 
            function(x,y,w,h,ctl,shift){self.handleSelectRect(x,y,w,h,ctl,shift);});

        this.autoAdjust=new AutoAdjust(this.boxOp, this.mouse, this.header);

       
        //this.projectiveViewOps.hide();
    
        if (!this.editorCfg.disableGrid)
            this.installGridLines()
    
        window.onbeforeunload = function() {
            return "Exit?";
            //if we return nothing here (just calling return;) then there will be no pop-up question at all
            //return;
        };

        this.onWindowResize();
    };



    this.run = function(){
        //this.animate();
        this.render();
        //$( "#maincanvas" ).resizable();
        
        
        this.imageContextManager.init_image_op(()=>this.selected_box);

        this.add_global_obj_type();        
    };

    this.hide = function(){
        this.wrapperUi.style.display="none";
    };
    this.show = function(){
        this.wrapperUi.style.display="block";
    };




    this.moveRangeCircle = function(world){
        if (this.rangeCircle.parent){
            world.webglGroup.add(this.rangeCircle);
        }
    };

    this.addRangeCircle= function(){
        
        var h = 1;
                        
        var body = [
        ];
        
        var segments=64;
        for (var i = 0; i<segments; i++){
            var theta1 = (2*Math.PI/segments) * i;
            var x1 = Math.cos(theta1);
            var y1 = Math.sin(theta1);

            var theta2 = 2*Math.PI/segments * ((i+1)%segments);
            var x2 = Math.cos(theta2);
            var y2 = Math.sin(theta2);

            body.push(x1,y1,h,x2,y2,h);
            body.push(0.6*x1,0.6*y1,h,0.6*x2,0.6*y2,h);
            body.push(2.0*x1,2.0*y1,h,2.0*x2,2.0*y2,h);
        }

        this.data.dbg.alloc();
        var bbox = new THREE.BufferGeometry();
        bbox.setAttribute( 'position', new THREE.Float32BufferAttribute(body, 3 ) );
        
        var box = new THREE.LineSegments( bbox, 
            new THREE.LineBasicMaterial( { color: 0x888800, linewidth: 1, opacity: 0.5, transparent: true } ) );    
         
        box.scale.x=50;
        box.scale.y=50;
        box.scale.z=-3;
        box.position.x=0;
        box.position.y=0;
        box.position.z=0;
        box.computeLineDistances();
        this.rangeCircle = box;
        this.scene.add(box);
    };


    this.showRangeCircle = function(show){

        if (show){
            if (this.data.world)
            {
                this.data.world.webglGroup.add(this.rangeCircle);
            }
        }
        else 
        {
            if (this.rangeCircle.parent)
                this.rangeCircle.parent.remove(this.rangeCircle);
        }

        this.render();
    };

    this.hideGridLines = function(){
        var svg = this.editorUi.querySelector("#grid-lines-wrapper");
        svg.style.display="none";
    };
    this.showGridLines = function(){
        var svg = this.editorUi.querySelector("#grid-lines-wrapper");
        svg.style.display="";
    };
    this.installGridLines= function(){
        
        var svg = this.editorUi.querySelector("#grid-lines-wrapper");

        for (var i=1; i<10; i++){
            const line = document. createElementNS("http://www.w3.org/2000/svg", "line");
            line.setAttribute("x1", "0%");
            line.setAttribute("y1", String(i*10)+"%");
            line.setAttribute("x2", "100%");
            line.setAttribute("y2", String(i*10)+"%");
            line.setAttribute("class", "grid-line");
            svg.appendChild(line);
        }

        for (var i=1; i<10; i++){
            const line = document. createElementNS("http://www.w3.org/2000/svg", "line");
            line.setAttribute("y1", "0%");
            line.setAttribute("x1", String(i*10)+"%");
            line.setAttribute("y2", "100%");
            line.setAttribute("x2", String(i*10)+"%");
            line.setAttribute("class", "grid-line");
            svg.appendChild(line);
        }
        
    };

    this.handleFastToolEvent= function(event){

        let self = this;
        switch (event.currentTarget.id){
        case "label-del":
            self.remove_selected_box();
            self.header.updateModifiedStatus();
            break;
        case "label-gen-id":
            //self.autoAdjust.mark_bbox(self.selected_box);
            //event.currentTarget.blur();
            let id = objIdManager.generateNewUniqueId();
            self.fastToolBox.setValue(self.selected_box.obj_type, id, self.selected_box.obj_attr);

            self.setObjectId(id);
            break;
        case "label-copy":
            if (!this.selected_box.obj_track_id)
            {
                this.infoBox.show("Error", "Please assign object track ID.");
            
            }
            else
            {
                self.autoAdjust.mark_bbox(self.selected_box);
            }
            break;

        case "label-paste":
            //this.autoAdjust.smart_paste(self.selected_box, null, (b)=>this.on_box_changed(b));
            this.boxOp.auto_rotate_xyz(this.selected_box, null, null, 
                (b)=>this.on_box_changed(b),
                "noscaling");
            //event.currentTarget.blur();
           break;

        case "label-batchedit":
            {

                if (!this.ensureBoxTrackIdExist())
                    break;

                if (!this.ensurePreloaded())
                    break;
                    
                this.header.setObject(this.selected_box.obj_track_id);
                this.editBatch(
                    this.data.world.frameInfo.scene,
                    this.data.world.frameInfo.frame,
                    this.selected_box.obj_track_id,
                    this.selected_box.obj_type
                );
            }
            break;


        case "label-trajectory":
            this.showTrajectory();            
            break;

        case "label-propagate":
            event.currentTarget.blur();
            this.openStackAnnotationPanel();
            break;

        case "label-edit":
            event.currentTarget.blur();
            self.selectBox(self.selected_box);
            break;
        

        // case "label-reset":
        //     event.currentTarget.blur();
        //     if (self.selected_box){
        //         //switch_bbox_type(this.selected_box.obj_type);
        //         self.transform_bbox("reset");
        //     }        
        //     break;

        case "label-highlight":
            event.currentTarget.blur();
            if (self.selected_box.in_highlight){
                self.cancelFocus(self.selected_box);
                self.view_state.lock_obj_in_highlight = false
            }
            else {
                self.focusOnSelectedBox(self.selected_box);
            }
            break;

        case "label-rotate":
            event.currentTarget.blur();
            self.transform_bbox("z_rotate_reverse");
            break;    

        case "label-rotate-90":
            event.currentTarget.blur();
            self.transform_bbox("z_rotate_right_angle");
            break;
        
        case "object-category-selector":
            this.object_category_changed(event);
            break;
        case "object-track-id-editor":
            this.object_track_id_changed(event);
            break;
        case "attr-input":
            this.object_attribute_changed(event.currentTarget.value);
            break;
        default:
            this.handleContextMenuEvent(event);
            break;   
        }

    };

    this.cancelFocus= function(box){
        
        box.in_highlight = false;
        //view_state.lock_obj_in_highlight = false; // when user unhighlight explicitly, set it to false
        this.data.world.lidar.cancel_highlight(box);
        this.floatLabelManager.restore_all();
        
        this.viewManager.mainView.save_orbit_state(box.scale);
        this.viewManager.mainView.orbit.reset();
    };

    this.focusOnSelectedBox = function(box){
        if (this.editorCfg.disableMainView)
            return;

        if (box){
            this.data.world.lidar.highlight_box_points(box);
            
            this.floatLabelManager.hide_all();
            this.viewManager.mainView.orbit.saveState();

            //this.viewManager.mainView.camera.position.set(this.selected_box.position.x+this.selected_box.scale.x*3, this.selected_box.position.y+this.selected_box.scale.y*3, this.selected_box.position.z+this.selected_box.scale.z*3);

            let posG = this.data.world.lidarPosToScene(box.position);
            this.viewManager.mainView.orbit.target.x = posG.x;
            this.viewManager.mainView.orbit.target.y = posG.y;
            this.viewManager.mainView.orbit.target.z = posG.z;

            this.viewManager.mainView.restore_relative_orbit_state(box.scale);
            this.viewManager.mainView.orbit.update();

            this.render();
            box.in_highlight=true;
            this.view_state.lock_obj_in_highlight = true;
        }
    };
    
    this.showTrajectory = function(){

        if (!this.selected_box)
            return;
            
        if (!this.selected_box.obj_track_id){
            console.error("no track id");
            return;
        }

        let tracks = this.data.worldList.map(w=>{
            let box = w.annotation.findBoxByTrackId(this.selected_box.obj_track_id);
            let ann = null;
            if (box){
                ann = w.annotation.boxToAnn(box);
                ann.psr.position = w.lidarPosToUtm(ann.psr.position);
                ann.psr.rotation = w.lidarRotToUtm(ann.psr.rotation);
            } 
            return [w.frameInfo.frame, ann, w===this.data.world]
        });


        tracks.sort((a,b)=> (a[0] > b[0])? 1 : -1);

        this.objectTrackView.setObject(
            this.selected_box.obj_type,
            this.selected_box.obj_track_id,
            tracks,
            (targetFrame)=>{  //onExit
                this.load_world(this.data.world.frameInfo.scene, targetFrame);
            }
        );
    }

    // return true to close contextmenu
    // return false to keep contextmenu
    this.handleContextMenuEvent = function(event){

        switch(event.currentTarget.id)
        {

        case "cm-play-2fps":
            this.playControl.play((w)=>{this.on_load_world_finished(w)}, 2);
            break;
        case "cm-play-10fps":
            this.playControl.play((w)=>{this.on_load_world_finished(w)}, 10);
            break;
        case "cm-play-20fps":
            this.playControl.play((w)=>{this.on_load_world_finished(w)}, 20);
            break;
        case "cm-play-50fps":
            this.playControl.play((w)=>{this.on_load_world_finished(w)}, 50);
            break;
        case 'cm-paste':
            {
                let box = this.add_box_on_mouse_pos_by_ref();

                if (!event.shiftKey)
                {
                    logger.log('paste without auto-adjusting');
                    this.boxOp.auto_rotate_xyz(box, null, null, 
                        b=>this.on_box_changed(b),
                        "noscaling");
                }
            }
            break;
        case 'cm-prev-frame':
            this.previous_frame();
            break;
        case 'cm-next-frame':
            this.next_frame();
            break;
        case 'cm-last-frame':
            this.last_frame();
            break;
        case 'cm-first-frame':
            this.first_frame();
            break;
        case 'cm-go-to-10hz':
            this.load_world(this.data.world.frameInfo.scene+"_10hz", this.data.world.frameInfo.frame)

            // {
            //     let link = document.createElement("a");
            //     //link.download=`${this.data.world.frameInfo.scene}-${this.data.world.frameInfo.frame}-webgl`;
            //     link.href="http://localhost";
            //     link.target="_blank";
            //     link.click();
            // }
            break;
        case 'cm-go-to-full-2hz':
            this.load_world(this.data.world.frameInfo.scene+"_full_2hz", this.data.world.frameInfo.frame)
            break;

        case 'cm-go-to-2hz':
            this.load_world(this.data.world.frameInfo.scene.split("_")[0], this.data.world.frameInfo.frame)
            break;
        

            
        case 'cm-save':
            saveWorldList(this.data.worldList);
            break;
       
        case "cm-reload":
            {
                reloadWorldList([this.data.world], ()=>{
                    this.on_load_world_finished(this.data.world);
                    this.header.updateModifiedStatus();
                });
                
            }
            break;

        case "cm-reload-all":
            {
                let modifiedFrames = this.data.worldList.filter(w=>w.annotation.modified);

                if (modifiedFrames.length > 0)
                {
                    this.infoBox.show(
                        "Confirm",
                        `Discard changes to ${modifiedFrames.length} frames, continue to reload?`,
                        ["yes","no"],
                        (choice)=>{
                            if (choice=="yes")
                            {
                                reloadWorldList(this.data.worldList, ()=>{
                                    this.on_load_world_finished(this.data.world);
                                    this.header.updateModifiedStatus();
                                });
                            }
                        }
                    );                
                }
                else
                {
                    reloadWorldList(this.data.worldList, ()=>{
                        this.on_load_world_finished(this.data.world);
                        this.header.updateModifiedStatus();
                    });

                    objIdManager.forceUpdate();
                }
            }
            break;
    

        case "cm-stop":
            this.playControl.stop_play();
            break;
        case "cm-pause":
            this.playControl.pause_resume_play();
            break;
        
        case "cm-prev-object":
            this.select_previous_object();
            break;
    
        case "cm-next-object":
            this.select_previous_object();
            break;

        case "cm-show-frame-info":
            {
                let info = {"scend-id": this.data.world.frameInfo.scene,
                            "frame": this.data.world.frameInfo.frame
                           };
                
                if (this.data.world.frameInfo.sceneMeta.desc)
                {
                    info = {
                        ...info, 
                        ...this.data.world.frameInfo.sceneMeta.desc,                        
                    };
                }

                this.infoBox.show("Frame info - " + this.data.world.frameInfo.scene, JSON.stringify(info,null,"<br>"));
            }
            break;

        case "cm-show-stat":
            {
                let scene = this.data.world.frameInfo.scene;
                objIdManager.load_obj_ids_of_scene(scene, (objs)=>{
                    let info = {
                        objects: objs.length,
                        boxes: objs.reduce((a,b)=>a+b.count, 0),
                        frames: this.data.world.frameInfo.sceneMeta.frames.length,
                    };

                    this.infoBox.show("Stat - " + scene, JSON.stringify(info, null,"<br>"));
                });
            }
            break;
        /// object

        case 'cm-check-scene':
            {
                let scene = this.data.world.frameInfo.scene;
                checkScene(scene);   
                logger.show();             
                logger.errorBtn.onclick();
            }
            break;
        case "cm-reset-view":
            this.resetView();
            break;
        case "cm-delete":
            this.remove_selected_box();
            this.header.updateModifiedStatus();
            break;

        case "cm-edit-multiple-instances":
            this.enterBatchEditMode();
            
            break;
        case "cm-auto-ann-background":
            {
                this.autoAnnInBackground();
               
            }
            break;
        case "cm-interpolate-background":
            {
                this.interpolateInBackground();
            }
            break;
        case "cm-show-trajectory":

            this.showTrajectory();
            break;
        case "cm-fit-moving-direction-by-id":
            this.fitMovingDirectionByTrackId();
            break;

        case "cm-select-as-ref":
            if (!this.selected_box.obj_track_id)
            {
                this.infoBox.show("Error", "Please assign object track ID.");
                return false;
            }
            else
            {
                this.autoAdjust.mark_bbox(this.selected_box);
            }
            break;
        
        case "cm-change-id-to-ref":
            if (!this.ensureRefObjExist())
                break;

            this.setObjectId(this.autoAdjust.marked_object.ann.obj_id);
            this.fastToolBox.setValue(this.selected_box.obj_type, 
                this.selected_box.obj_track_id, 
                this.selected_box.obj_attr);

            break;
        case "cm-change-id-to-ref-in-scene":

            if (!this.ensureBoxTrackIdExist())
                break;
            if (!this.ensurePreloaded())
                break;
            if (!this.ensureRefObjExist())
                break;

            this.data.worldList.forEach(w=>{
                let box = w.annotation.boxes.find(b=>b.obj_track_id === this.selected_box.obj_track_id &&  b.obj_type === this.selected_box.obj_type);
                if (box && box !== this.selected_box){
                    box.obj_track_id = this.autoAdjust.marked_object.ann.obj_id;
                    w.annotation.setModified();
                }
            });

            
            this.setObjectId(this.autoAdjust.marked_object.ann.obj_id);
            this.fastToolBox.setValue(this.selected_box.obj_type, 
                this.selected_box.obj_track_id, 
                this.selected_box.obj_attr);

            break;
        case "cm-follow-ref":

            if (!this.ensureBoxTrackIdExist())
                break;
            if (!this.ensurePreloaded())
                break;
            this.autoAdjust.followsRef(this.selected_box).then(()=>{
                this.header.updateModifiedStatus();
                this.editBatch(
                    this.data.world.frameInfo.scene,
                    this.data.world.frameInfo.frame,
                    this.selected_box.obj_track_id,
                    this.selected_box.obj_type
                );
            }).catch(error=>{
                console.error("follow ref failed", error);
            });
            break;
        case 'cm-follow-static-objects':
            if (!this.ensureBoxTrackIdExist())
                break;
            if (!this.ensurePreloaded())
                break;
            this.autoAdjust.followStaticObjects(this.selected_box).then(()=>{
                this.header.updateModifiedStatus();

                this.editBatch(
                    this.data.world.frameInfo.scene,
                    this.data.world.frameInfo.frame,
                    this.selected_box.obj_track_id,
                    this.selected_box.obj_type
                );
            }).catch(error=>{
                console.error("follow static objects failed", error);
            });

            break;
        case "cm-sync-followers":
            
            if (!this.ensurePreloaded())
                break;
            this.autoAdjust.syncFollowers(this.selected_box);
            this.header.updateModifiedStatus();
            this.render();
            break;


        case "cm-delete-obj-range":
            this.deleteSelectedObjectRange();
            return true;

        case "cm-delete-obj":
            this.deleteSelectedObjectRange(null);
            return true;

        case "cm-delete-obj-all":
            this.deleteSelectedObjectInAllFrames();
            return true;

        case "cm-rotate-obj-yaw-90-all":
            this.rotateSelectedObjectYaw90InAllFrames();
            return true;

        case "cm-rotate-box-yaw-90-swap":
            this.rotateSelectedBoxYaw90AndSwapSize();
            return true;

        case "cm-copy-obj-next-frames":
            this.copySelectedObjectToNextFrames();
            return true;

        case "cm-modify-obj-type":
            {
                if (!this.ensurePreloaded())
                    break;
                //let saveList=[];
                this.data.worldList.forEach(w=>{
                    let box = w.annotation.boxes.find(b=>b.obj_track_id === this.selected_box.obj_track_id);
                    if (box && box !== this.selected_box){
                        box.obj_type = this.selected_box.obj_type;
                        box.obj_attr = this.selected_box.obj_attr;
                        //saveList.push(w);
                        w.annotation.setModified();
                    }                
                    
                });

                //saveWorldList(saveList);
                this.header.updateModifiedStatus();
            }
            break;

        case "cm-modify-obj-size":
            {
                if (!this.ensurePreloaded())
                    break;
                //let saveList=[];
                this.data.worldList.forEach(w=>{
                    let box = w.annotation.boxes.find(b=>b.obj_track_id == this.selected_box.obj_track_id);
                    if (box && box !== this.selected_box){
                        box.scale.x = this.selected_box.scale.x;
                        box.scale.y = this.selected_box.scale.y;
                        box.scale.z = this.selected_box.scale.z;
                        //saveList.push(w);

                        w.annotation.setModified();
                    }                
                    
                });

                //saveWorldList(saveList);
                this.header.updateModifiedStatus();
            }
            break;


        default:
            console.log('unhandled', event.currentTarget.id, event.type);
        }

        return true; 
    };

    // this.animate= function() {
    //     let self=this;
    //     requestAnimationFrame( function(){self.animate();} );
    //     this.viewManager.mainView.orbit_orth.update();
    // };



    this.render= function(){

        this.viewManager.mainView.render();
        this.boxEditor.boxView.render();

        this.floatLabelManager.update_all_position();
        if (this.selected_box){
            this.fastToolBox.setPos(this.floatLabelManager.getLabelEditorPos(this.selected_box.obj_local_id));
        }
    };

    
    this.resetView = function(targetPos){

        if (!targetPos){
            let center = this.data.world.lidar.computeCenter();
            targetPos = {...center};//{x:0, y:0, z:50};
            targetPos.z += 50;
        }
        else
            targetPos.z = 50;

        let pos = this.data.world.lidarPosToScene(targetPos);
        this.viewManager.mainView.orbit.object.position.set(pos.x, pos.y, pos.z);  //object is camera
        this.viewManager.mainView.orbit.target.set(pos.x, pos.y, 0);
        this.viewManager.mainView.orbit.update(); 
        this.render();
    };

    this.scene_changed= async function(sceneName){
        
        //var sceneName = event.currentTarget.value;

        if (sceneName.length == 0){
            return;
        }
        
        console.log("choose sceneName " + sceneName);
        var meta = this.data.getMetaBySceneName(sceneName);

        if (!meta)
        {
            this.editorUi.querySelector("#frame-selector").innerHTML = "<option>--frame--</option>";
            meta = await this.data.readSceneMetaData(sceneName);
        }

        var frame_selector_str = meta.frames.map(function(f){
            return "<option value="+f+">"+f + "</option>";
        }).reduce(function(x,y){return x+y;}, "<option>--frame--</option>");

        this.editorUi.querySelector("#frame-selector").innerHTML = frame_selector_str;
        
        
        if (meta.camera){
            this.imageContextManager.updateCameraList(meta.camera);
        }

        //load_obj_ids_of_scene(sceneName);
    };

    this.frame_changed= function(event){
        var sceneName = this.editorUi.querySelector("#scene-selector").value;

        if (sceneName.length == 0 && this.data.world)
        {
            sceneName = this.data.world.frameInfo.scene;
        }

        if (sceneName.length == 0){
            return;
        }

        var frame =  event.currentTarget.value;
        console.log(sceneName, frame);
        this.load_world(sceneName, frame);        
        event.currentTarget.blur();
    };


    this.ensureBoxTrackIdExist = function()
    {
        if (!this.selected_box.obj_track_id)
        {
            this.infoBox.show("Error", "Please assign object track ID.");
            return false;
        }

        return true;
    }

    this.ensureRefObjExist = function()
    {
        if (!this.autoAdjust.marked_object)
        {
            this.infoBox.show("Notice", 'No reference object was selected');
            return false;
        }

        
        return true;
    }
    this.ensurePreloaded = function()
    {
        let worldList = this.data.worldList.filter(w=>w.frameInfo.scene == this.data.world.frameInfo.scene);
        worldList = worldList.sort((a,b)=>a.frameInfo.frame_index - b.frameInfo.frame_index);

        let meta = this.data.get_current_world_scene_meta();

        
        let allLoaded = worldList.map(w=>w.preloaded()).reduce((a,b)=>a && b, true);

        if ((worldList.length < meta.frames.length && worldList.length <= 60) || (!allLoaded))
        {
            this.data.forcePreloadScene(this.data.world.frameInfo.scene, this.data.world);

            this.infoBox.show("Notice", 
                `Loading scene in background. Please try again later.`);
            return false;
        }

        
        return true;
    }


    this.interpolateInBackground = function()
    {
        
        if (!this.ensureBoxTrackIdExist())
        return;

        if (!this.ensurePreloaded())
            return;

        let worldList = this.data.worldList.filter(w=>w.frameInfo.scene == this.data.world.frameInfo.scene);
        worldList = worldList.sort((a,b)=>a.frameInfo.frame_index - b.frameInfo.frame_index);
        let boxList = worldList.map(w=>w.annotation.findBoxByTrackId(this.selected_box.obj_track_id));

        let applyIndList = boxList.map(b=>true);
        this.boxOp.interpolateAsync(worldList, boxList, applyIndList).then(ret=>{
            this.header.updateModifiedStatus();
            this.viewManager.render();
        });
    };
    this.enterBatchEditMode = function()
    {
        if (!this.ensureBoxTrackIdExist())
           return;

        if (!this.ensurePreloaded())
            return;

        this.header.setObject(this.selected_box.obj_track_id);

        this.editBatch(
            this.data.world.frameInfo.scene,
            this.data.world.frameInfo.frame,
            this.selected_box.obj_track_id,
            this.selected_box.obj_type
        );
    };

    this.autoAnnInBackground = function()
    {
        if (!this.ensureBoxTrackIdExist())
            return;

        if (!this.ensurePreloaded())
            return;

        let worldList = this.data.worldList.filter(w=>w.frameInfo.scene == this.data.world.frameInfo.scene);
        worldList = worldList.sort((a,b)=>a.frameInfo.frame_index - b.frameInfo.frame_index);



        let boxList = worldList.map(w=>w.annotation.findBoxByTrackId(this.selected_box.obj_track_id));

        let onFinishOneBox = (i)=>{
            this.viewManager.render();
        }
        let applyIndList = boxList.map(b=>true);
        let dontRotate = false;

        this.boxOp.interpolateAndAutoAdjustAsync(worldList, boxList, onFinishOneBox, applyIndList, dontRotate).then(ret=>{
            this.header.updateModifiedStatus();
        });
    };


    this.fitMovingDirectionByTrackId = async function()
    {
        if (!this.ensureBoxTrackIdExist())
            return;

        const scene = this.data.world.frameInfo.scene;
        const objId = this.selected_box.obj_track_id;
        const modifiedFrames = this.data.worldList.filter(w=>w.frameInfo.scene === scene && w.annotation.modified);

        try{
            if (modifiedFrames.length > 0){
                await saveWorldListImmediate(modifiedFrames);
            }

            const params = new URLSearchParams({
                scene: scene,
                obj_id: objId,
            });
            const response = await fetch("/fit_moving_direction_by_id?" + params.toString());
            const result = await response.json();

            if (!response.ok || result.error){
                throw new Error(result.error || `HTTP error ${response.status}`);
            }

            this.onAnnotationUpdatedByOthers(scene, result.updated_frames || []);
            this.render();

            const updatedCount = result.updated_count ?? ((result.updated_frames || []).length);
            const fittedCount = result.fitted_count ?? ((result.fitted_frames || []).length);
            const skippedCount = result.skipped_count ?? ((result.skipped_frames || []).length);

            this.infoBox.show(
                "Notice",
                `Moving-direction heading fit finished.<br>` +
                `Updated: ${updatedCount}<br>` +
                `Fitted: ${fittedCount}<br>` +
                `Skipped: ${skippedCount}<br>` +
                `${result.message || ""}`
            );
        }catch(error){
            console.error("fit moving direction by id failed", error);
            this.infoBox.show("Error", `Fit moving direction failed: ${error.message}`);
        }
    };

    this.deleteSelectedObjectRange = async function(count){
        if (!this.selected_box){
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        if (!this.ensureBoxTrackIdExist())
            return;

        const scene = this.data.world.frameInfo.scene;
        const startFrame = this.data.world.frameInfo.frame;
        const objId = this.selected_box.obj_track_id;

        if (count === undefined){
            const raw = window.prompt("Delete this object from current frame for how many frames?", "1");
            if (raw === null)
                return;

            count = parseInt(raw, 10);
            if (!Number.isInteger(count) || count <= 0){
                this.infoBox.show("Error", "Frame count must be a positive integer.");
                return;
            }
        }

        const rangeText = count === null ? "all following frames" : `${count} frame(s)`;
        const confirmed = window.confirm(
            `Delete object ${objId} from frame ${startFrame} for ${rangeText}?`
        );
        if (!confirmed)
            return;

        try{
            const modifiedFrames = this.data.worldList.filter(w=>
                w.frameInfo.scene === scene && w.annotation.modified
            );
            if (modifiedFrames.length > 0){
                await saveWorldListImmediate(modifiedFrames);
            }

            const result = await this._postJson("/delete_object_range", {
                scene: scene,
                start_frame: startFrame,
                obj_id: objId,
                count: count,
            });

            if (result.error)
                throw new Error(result.error);

            const updatedFrames = result.updated_frames || [];
            const backup = result.backup || [];

            if (backup.length > 0){
                this.batchUndoStack.push({
                    scene: scene,
                    frames: updatedFrames,
                    backup: backup,
                    objId: objId,
                });
            }

            await this.reloadAnnotationsFromDisk(scene, updatedFrames);

            this.infoBox.show(
                "Notice",
                `Deleted object ${objId}.<br>` +
                `Updated frames: ${updatedFrames.length}.`
            );
        } catch(error) {
            console.error("delete object range failed", error);
            this.infoBox.show("Error", `Delete failed: ${error.message}`);
        }
    };

    this.deleteSelectedObjectInAllFrames = async function(){
        if (!this.selected_box){
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        if (!this.ensureBoxTrackIdExist())
            return;

        const scene = this.data.world.frameInfo.scene;
        const meta = this.data.get_current_world_scene_meta();
        const firstFrame = meta && meta.frames && meta.frames.length > 0 ? meta.frames[0] : this.data.world.frameInfo.frame;
        const objId = this.selected_box.obj_track_id;

        const confirmed = window.confirm(
            `Delete object ${objId} from all frames in scene ${scene}?`
        );
        if (!confirmed)
            return;

        try{
            const modifiedFrames = this.data.worldList.filter(w=>
                w.frameInfo.scene === scene && w.annotation.modified
            );
            if (modifiedFrames.length > 0){
                await saveWorldListImmediate(modifiedFrames);
            }

            const result = await this._postJson("/delete_object_range", {
                scene: scene,
                start_frame: firstFrame,
                obj_id: objId,
                count: null,
            });

            if (result.error)
                throw new Error(result.error);

            const updatedFrames = result.updated_frames || [];
            const backup = result.backup || [];

            if (backup.length > 0){
                this.batchUndoStack.push({
                    type: "delete",
                    scene: scene,
                    frames: updatedFrames,
                    backup: backup,
                    objId: objId,
                });
            }

            await this.reloadAnnotationsFromDisk(scene, updatedFrames);

            this.infoBox.show(
                "Notice",
                `Deleted object ${objId} from all frames.<br>` +
                `Updated frames: ${updatedFrames.length}.`
            );
        } catch(error) {
            console.error("delete object all frames failed", error);
            this.infoBox.show("Error", `Delete failed: ${error.message}`);
        }
    };

    this.rotateSelectedObjectYaw90InAllFrames = async function(){
        if (!this.selected_box){
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        if (!this.ensureBoxTrackIdExist())
            return;

        const scene = this.data.world.frameInfo.scene;
        const objId = this.selected_box.obj_track_id;
        const angleDelta = Math.PI / 2;

        const confirmed = window.confirm(
            `Rotate right 90 degrees and swap length/width for object ${objId} in all frames of scene ${scene}?`
        );
        if (!confirmed)
            return;

        try{
            const modifiedFrames = this.data.worldList.filter(w=>
                w.frameInfo.scene === scene && w.annotation.modified
            );
            if (modifiedFrames.length > 0){
                await saveWorldListImmediate(modifiedFrames);
            }

            const result = await this._postJson("/rotate_object_yaw_all", {
                scene: scene,
                obj_id: objId,
                angle_delta: -angleDelta,
                swap_xy: true,
            });

            if (result.error)
                throw new Error(result.error);

            const updatedFrames = result.updated_frames || [];
            const backup = result.backup || [];

            if (backup.length > 0){
                this.batchUndoStack.push({
                    type: "rotate-yaw",
                    scene: scene,
                    frames: updatedFrames,
                    backup: backup,
                    objId: objId,
                });
            }

            await this.reloadAnnotationsFromDisk(scene, updatedFrames);

            this.infoBox.show(
                "Notice",
                `Rotated object ${objId} right 90 degrees and swapped length/width in all frames.<br>` +
                `Updated frames: ${updatedFrames.length}.`
            );
        } catch(error) {
            console.error("rotate object yaw all frames failed", error);
            this.infoBox.show("Error", `Rotate yaw failed: ${error.message}`);
        }
    };

    this.rotateSelectedBoxYaw90AndSwapSize = function(){
        if (!this.selected_box){
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        if (this.data.world && this.data.world.annotation) {
            this.data.world.annotation.undoManager.takeSnapshot(this.data.world.annotation);
        }

        this.selected_box.rotation.z -= Math.PI / 2;
        this.on_box_changed(this.selected_box);
    };

    this.copySelectedObjectToNextFrames = async function(){
        if (!this.selected_box){
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        if (!this.ensureBoxTrackIdExist())
            return;

        const raw = window.prompt("Copy this object to how many next frames?", "1");
        if (raw === null)
            return;

        const count = parseInt(raw, 10);
        if (!Number.isInteger(count) || count <= 0){
            this.infoBox.show("Error", "Frame count must be a positive integer.");
            return;
        }

        const scene = this.data.world.frameInfo.scene;
        const startFrame = this.data.world.frameInfo.frame;
        const sourceAnn = this.data.world.annotation.boxToAnn(this.selected_box);
        const objId = sourceAnn.obj_id;

        const confirmed = window.confirm(
            `Copy object ${objId} from frame ${startFrame} to next ${count} frame(s)? Existing same-id boxes will be overwritten.`
        );
        if (!confirmed)
            return;

        try{
            const modifiedFrames = this.data.worldList.filter(w=>
                w.frameInfo.scene === scene && w.annotation.modified
            );
            if (modifiedFrames.length > 0){
                await saveWorldListImmediate(modifiedFrames);
            }

            const result = await this._postJson("/copy_object_to_next_frames", {
                scene: scene,
                start_frame: startFrame,
                count: count,
                annotation: sourceAnn,
            });

            if (result.error)
                throw new Error(result.error);

            const updatedFrames = result.updated_frames || [];
            const backup = result.backup || [];

            if (backup.length > 0){
                this.batchUndoStack.push({
                    type: "copy-static",
                    scene: scene,
                    frames: updatedFrames,
                    backup: backup,
                    objId: objId,
                });
            }

            await this.reloadAnnotationsFromDisk(scene, updatedFrames);

            this.infoBox.show(
                "Notice",
                `Copied object ${objId} to next frames.<br>` +
                `Updated frames: ${updatedFrames.length}.`
            );
        } catch(error) {
            console.error("copy object to next frames failed", error);
            this.infoBox.show("Error", `Copy failed: ${error.message}`);
        }
    };

    this._postJson = async function(url, data){
        const response = await fetch(url, {
            method: "POST",
            headers: {"Content-Type": "application/json"},
            body: JSON.stringify(data),
        });

        const result = await response.json();
        if (!response.ok){
            throw new Error(result.error || `HTTP error ${response.status}`);
        }
        return result;
    };

    this.restoreLastBatchDelete = async function(){
        if (!this.batchUndoStack || this.batchUndoStack.length === 0)
            return false;

        const entry = this.batchUndoStack[this.batchUndoStack.length - 1];

        try{
            const response = await fetch("/saveworldlist", {
                method: "POST",
                headers: {"Content-Type": "application/json"},
                body: JSON.stringify(entry.backup),
            });

            if (!response.ok)
                throw new Error(`HTTP error ${response.status}`);

            this.batchUndoStack.pop();
            await this.reloadAnnotationsFromDisk(entry.scene, entry.frames);

            const actionText = {
                "rotate-yaw": "yaw rotation",
                "copy-static": "static-object copy",
            }[entry.type] || "object delete";
            this.infoBox.show(
                "Notice",
                `Restored ${actionText} for object ${entry.objId}.<br>` +
                `Restored frames: ${entry.frames.length}.`
            );
            return true;
        } catch(error) {
            console.error("restore batch delete failed", error);
            this.infoBox.show("Error", `Undo failed: ${error.message}`);
            return true;
        }
    };

    this.reloadAnnotationsFromDisk = async function(scene, frames){
        if (!frames || frames.length === 0){
            this.header.updateModifiedStatus();
            this.render();
            return;
        }

        const currentWorld = this.data.world;
        const currentFrameAffected = currentWorld &&
            currentWorld.frameInfo.scene === scene &&
            frames.includes(currentWorld.frameInfo.frame);

        if (currentFrameAffected && this.selected_box){
            this.unselectBox(null, true);
            this.unselectBox(null, true);
            this.selected_box = null;
        }

        await this.onAnnotationUpdatedByOthers(scene, frames);

        if (currentFrameAffected){
            this.imageContextManager.attachWorld(currentWorld);
            this.imageContextManager.render_2d_image();
            this.render2dLabels(currentWorld);
            this.update_frame_info(currentWorld.frameInfo.scene, currentWorld.frameInfo.frame);
        }

        this.header.updateModifiedStatus();
        this.render();
    };


    this.editBatch = function(sceneName, frame, objectTrackId, objectType){

        

        //this.keydownDisabled = true;
        // hide something
        this.imageContextManager.hide();
        this.floatLabelManager.hide();

        //this.floatLabelManager.showFastToolbox();

        this.viewManager.mainView.disable();
        this.boxEditor.hide();
        this.hideGridLines();
        //this.controlGui.hide();
        this.editorUi.querySelector("#selectors").style.display='none';
        //this.editorUi.querySelector("#object-selector").style.display='none';
        this.currentMainEditor = this.boxEditorManager;

        this.boxEditorManager.edit(this.data, 
            this.data.getMetaBySceneName(sceneName), 
            frame, 
            objectTrackId,
            objectType,
            (targetFrame, targetTrackId)=>{  //on exit
                this.currentMainEditor = this
                //this.keydownDisabled = false;
                this.viewManager.mainView.enable();

                this.imageContextManager.show();
                this.floatLabelManager.show();

                if (targetTrackId)
                    this.view_state.lock_obj_track_id = targetTrackId;

                this.on_load_world_finished(this.data.world);
                
                // if (this.selected_box){
                //     // attach again, restore box.boxEditor 
                //     // obj type/id may have changed in batch mode
                //     this.floatLabelManager.set_object_track_id(this.selected_box.obj_local_id, this.selected_box.obj_track_id);
                //     this.boxEditor.attachBox(this.selected_box);
                //     this.boxEditor.update();

                //     // update fasttoolbox
                //     this.fastToolBox.setValue(this.selected_box.obj_type, this.selected_box.obj_track_id, this.selected_box.obj_attr);
                // }

                
                this.showGridLines();
                this.render();
                //this.controlGui.show();
                this.editorUi.querySelector("#selectors").style.display='inherit';

                if (targetFrame)
                {
                    this.load_world(this.data.world.frameInfo.scene, targetFrame, ()=>{  // onfinished
                        this.makeVisible(targetTrackId);
                    });
                }
            }
            );
    };

    this.gotoObjectFrame = function(frame, objId)
    {
        this.load_world(this.data.world.frameInfo.scene, frame, ()=>{  // onfinished
            this.makeVisible(objId);
        });
    };

    this.makeVisible = function(targetTrackId){
        let box = this.data.world.annotation.findBoxByTrackId(targetTrackId);

        if (box){
            if (this.selected_box != box){
                this.selectBox(box);
            }

            this.resetView({x:box.position.x, y:box.position.y, z:50});
        }

    };

    this.object_changed = function(event){
        var sceneName = this.data.world.frameInfo.scene; //this.editorUi.querySelector("#scene-selector").value;

        let objectTrackId = event.currentTarget.value;
        let obj = objIdManager.getObjById(objectTrackId);

        this.editBatch(sceneName, null, objectTrackId, obj.category);
    };

    this.camera_changed= function(event){
        var camera_name = event.currentTarget.value;

        this.data.set_active_image(camera_name);
        this.imageContextManager.render_2d_image();

        event.currentTarget.blur();
    };

    this.downloadWebglScreenShot = function(){
        let link = document.createElement("a");
        link.download=`${this.data.world.frameInfo.scene}-${this.data.world.frameInfo.frame}-webgl`;
        link.href=this.renderer.domElement.toDataURL("image/png", 1);
        link.click();
    };

    this.showLog = function() {

    };

    this.annotateByAlg1 = function(){
        autoAnnotate(this.data.world, ()=>this.on_load_world_finished(this.data.world));
    };



    this.object_category_changed= function(event){
        if (this.selected_box){
            
            let category = event.currentTarget.value;

            this.selected_box.obj_type = category;
            this.floatLabelManager.set_object_type(this.selected_box.obj_local_id, this.selected_box.obj_type);
            // this.header.mark_changed_flag();
            // this.updateBoxPointsColor(this.selected_box);
            // this.imageContextManager.boxes_manager.update_obj_type(this.selected_box.obj_local_id, this.selected_box.obj_type);

            // this.render();
            this.on_box_changed(this.selected_box);

            //todo: we don't know if the old one is already deleted.
            // could use object count number?
            objIdManager.addObject({
                category: this.selected_box.obj_type,
                id: this.selected_box.obj_track_id,
            });

        }
    };

    this.setObjectId = function(id)
    {
        if (this.data.world && this.data.world.annotation) {
            this.data.world.annotation.undoManager.takeSnapshot(this.data.world.annotation);
        }
        this.selected_box.obj_track_id = id;
        this.floatLabelManager.set_object_track_id(this.selected_box.obj_local_id, this.selected_box.obj_track_id);

        this.view_state.lock_obj_track_id = id;

        //this.header.mark_changed_flag();
        this.on_box_changed(this.selected_box);

        //
        objIdManager.addObject({
            category: this.selected_box.obj_type,
            id: this.selected_box.obj_track_id,
        });
    }

    this.object_track_id_changed= function(event){
        if (this.selected_box){
            var id = event.currentTarget.value;
            this.setObjectId(id);            
        }
    };

    this.object_attribute_changed = function(value){
        if (this.selected_box){
            if (this.data.world && this.data.world.annotation) {
                this.data.world.annotation.undoManager.takeSnapshot(this.data.world.annotation);
            }
            this.selected_box.obj_attr = value;
            this.floatLabelManager.set_object_attr(this.selected_box.obj_local_id, value);
            this.data.world.annotation.setModified();
            this.header.updateModifiedStatus();
        }
    };

    // this.updateSubviewRangeByWindowResize= function(box){

    //     if (box === null)
    //         return;

    //     if (box.boxEditor)
    //         box.boxEditor.onWindowResize();

    //     this.render();
    // };

    this.handleRightClick= function(event){

        // select new object

        if (!this.data.world){
            return;
        }


        if (event.shiftKey || event.ctrlKey)
        {
            // if ctrl or shift hold, don't select any object.
            this.contextMenu.show("world",event.layerX, event.layerY, this);
            return;
        }


        var intersects = this.mouse.getIntersects( this.mouse.onUpPosition, this.data.world.annotation.boxes );
        if ( intersects.length > 0 ) {
            //var object = intersects[ 0 ].object;
            var object = intersects[ 0 ].object;
            let target_obj = object.userData.object;
            if ( target_obj == undefined ) {
                // helper
                target_obj = object;
            }

            if (target_obj != this.selected_box){
                this.selectBox(target_obj);
            }

            // this.hide_world_context_menu();
            // this.show_object_context_menu(event.layerX, event.layerY);
            this.contextMenu.show("object",event.layerX, event.layerY, this);

        } else {
            // if no object is selected, popup context menu
            //var pos = getMousePosition(renderer.domElement, event.clientX, event.clientY );
            this.contextMenu.show("world",event.layerX, event.layerY, this);
        }
    };

    this.show_world_context_menu= function(posX, posY){
        let menu = this.editorUi.querySelector("#context-menu");
        menu.style.display = "inherit";
        menu.style.left = posX+"px";
        menu.style.top = posY+"px";
        this.editorUi.querySelector("#context-menu-wrapper").style.display = "block";
    };

    this.hide_world_context_menu= function(){
        let menu = this.editorUi.querySelector("#context-menu");
        menu.style.display = "none";
    };

    this.show_object_context_menu= function(posX, posY){
        let menu = this.editorUi.querySelector("#object-context-menu");
        menu.style.display = "inherit";
        menu.style.left = posX+"px";
        menu.style.top = posY+"px";
        this.editorUi.querySelector("#context-menu-wrapper").style.display = "block";
    };

    this.hide_object_context_menu= function(){
        let menu = this.editorUi.querySelector("#object-context-menu");
        menu.style.display = "none";
    };

    this.on_img_click = function(lidar_point_indices){
        
        console.log(lidar_point_indices);

        var self=this;
        let obj_type = "Car";
        this.data.world.lidar.set_spec_points_color(lidar_point_indices, {x:0,y:0,z:1});
        this.data.world.lidar.update_points_color();
        this.render();
        //return;

        let pos = this.data.world.lidar.get_centroid(lidar_point_indices);
        pos.z = 0;

        let rotation = {x:0, y:0, z:this.viewManager.mainView.camera.rotation.z+Math.PI/2};

        let obj_cfg = globalObjectCategory.get_obj_cfg_by_type(obj_type);
        let scale = {   
            x: obj_cfg.size[0],
            y: obj_cfg.size[1],
            z: obj_cfg.size[2]
        };

        let box = this.add_box(pos, scale, rotation, obj_type, "");
        self.boxOp.auto_rotate_xyz(box, null, null, function(b){
            self.on_box_changed(b);
        });

        return;
        /*
        var box = this.data.world.lidar.create_box_by_points(lidar_point_indices, this.viewManager.mainView.camera);
        

        this.scene.add(box);
        
        this.imageContextManager.boxes_manager.add_box(box);
        
        
        this.boxOp.auto_shrink_box(box);
        
        
        // guess obj type here
        
        box.obj_type = guess_obj_type_by_dimension(box.scale);
        
        this.floatLabelManager.add_label(box);

        this.selectBox(box);
        this.on_box_changed(box);

        
        this.boxOp.auto_rotate_xyz(box, function(){
            box.obj_type = guess_obj_type_by_dimension(box.scale);
            self.floatLabelManager.set_object_type(box.obj_local_id, box.obj_type);
            self.floatLabelManager.update_label_editor(box.obj_type, box.obj_track_id);
            self.on_box_changed(box);
        });
        */
        
    };
    
    this.handleSelectRect= function(x,y,w,h, ctrl, shift){
        // y = y+h;
        // x = x*2-1;
        // y = -y*2+1;
        // w *= 2;
        // h *= 2;
        
        // x,y: start cornor, w: width, h: height

        /*
        console.log("main select rect", x,y,w,h);

        this.viewManager.mainView.camera.updateProjectionMatrix();
        this.data.world.select_points_by_view_rect(x,y,w,h, this.viewManager.mainView.camera);
        render();
        render_2d_image();
        */

        // check if any box is inside the rectangle

        this.viewManager.mainView.camera.updateProjectionMatrix();

        let boxes = this.data.world.annotation.find_boxes_inside_rect(x,y,w,h, this.viewManager.mainView.camera);
        if (boxes.length > 0) {

            if (boxes.length == 1){
                this.selectBox(boxes[0])
            }
            else{
                // this is dangerous
                // for (let b in boxes){
                //     this.remove_box(boxes[b],false)
                // }
                // this.render();
            }

            return;
        }

        let points = this.data.world.lidar.select_points_by_view_rect(x,y,w,h, this.viewManager.mainView.camera);

        // show color
        //this.render();

        // return;
        // // create new box
        // var self=this;
        var center_pos = this.mouse.get_screen_location_in_world(x+w/2, y+h/2);
        center_pos = this.data.world.scenePosToLidar(center_pos);
        
        let initRoationZ = this.viewManager.mainView.camera.rotation.z + Math.PI/2;

        let box = this.create_box_by_points(points, initRoationZ);

        let id = objIdManager.generateNewUniqueId();
        box.obj_track_id = id;

        

        //this.scene.add(box);
        
        
        
        if (!shift){
            try{
                this.boxOp.auto_shrink_box(box);
            }
            catch(e)
            {
                logger.log(e);                
            }
        }
        
        // guess obj type here
        
        box.obj_type = globalObjectCategory.guess_obj_type_by_dimension(box.scale);
        
        objIdManager.addObject({
            category: box.obj_type,
            id: box.obj_track_id,
        });



        this.imageContextManager.boxes_manager.add_box(box);
        this.floatLabelManager.add_label(box);

        this.selectBox(box);
        this.on_box_changed(box);

        if (!shift){
            this.boxOp.auto_rotate_xyz(box, ()=>{
                box.obj_type = globalObjectCategory.guess_obj_type_by_dimension(box.scale);
                this.floatLabelManager.set_object_type(box.obj_local_id, box.obj_type);
                this.fastToolBox.setValue(box.obj_type, box.obj_track_id, box.obj_attr);
                this.on_box_changed(box);
            });
        }
        
        
        //floatLabelManager.add_label(box);

        
    };



    this.create_box_by_points=function(points, rotationZ){
        
        let localRot = this.data.world.sceneRotToLidar(new THREE.Euler(0,0,rotationZ, "XYZ"));
        
        let transToBoxMatrix = new THREE.Matrix4().makeRotationFromEuler(localRot)
                                                  .setPosition(0, 0, 0)
                                                  .invert();

       // var trans = transpose(euler_angle_to_rotate_matrix({x:0,y:0,z:rotation_z}, {x:0, y:0, z:0}), 4);

        let relative_position = [];
        let v = new THREE.Vector3();
        points.forEach(function(p){
            v.set(p[0],p[1],p[2]);
            let boxP = v.applyMatrix4(transToBoxMatrix);
            relative_position.push([boxP.x,boxP.y, boxP.z]);
        });

        var relative_extreme = vector_range(relative_position);
        var scale = {
            x: relative_extreme.max[0] - relative_extreme.min[0],
            y: relative_extreme.max[1] - relative_extreme.min[1],
            z: relative_extreme.max[2] - relative_extreme.min[2],
        };

        // enlarge scale a little

        let center = this.boxOp.translateBoxInBoxCoord(
            localRot,
            {
                x: (relative_extreme.max[0] + relative_extreme.min[0])/2,
                y: (relative_extreme.max[1] + relative_extreme.min[1])/2,
                z: (relative_extreme.max[2] + relative_extreme.min[2])/2,
            }
        );

        return this.data.world.annotation.add_box(center, scale, localRot, "Unknown", "");
    };


    this.handleLeftClick= function(event) {

            if (event.ctrlKey){
                //Ctrl+left click to smart paste!
                //smart_paste();
            }
            else{
                //select box /unselect box
                if (!this.data.world || (!this.data.world.annotation.boxes && this.data.world.radars.radarList.length==0 && !this.calib.calib_box)){
                    return;
                }

                let all_boxes = this.data.world.annotation.boxes.concat(this.data.world.radars.getAllBoxes());
                all_boxes = all_boxes.concat(this.data.world.aux_lidars.getAllBoxes());
                
                if (this.calib.calib_box){
                    all_boxes.push(this.calib.calib_box);
                }
                
                let intersects = this.mouse.getIntersects( this.mouse.onUpPosition, all_boxes);

                if (intersects.length == 0){
                    if (this.data.world.radar_box){
                        intersects = this.mouse.getIntersects( this.mouse.onUpPosition, [this.data.world.radar_box]);
                    }
                }

                if ( intersects.length > 0 ) {
                    //var object = intersects[ 0 ].object;
                    var object = intersects[ 0 ].object;
                    if ( object.userData.object !== undefined ) {
                        // helper
                        this.selectBox( object.userData.object );
                    } else {
                        this.selectBox( object );
                    }
                } else {
                    this.unselectBox(null);
                }

                //render();
            }
        

    };

    this.select_locked_object= function(){
        var self=this;
        if (this.view_state.lock_obj_track_id != ""){
            var box = this.data.world.annotation.boxes.find(function(x){
                return x.obj_track_id == self.view_state.lock_obj_track_id;
            })

            if (box){
                this.selectBox(box);

                if (self.view_state.lock_obj_in_highlight){
                    this.focusOnSelectedBox(box);
                }
            }
        }
    };

    // new_object
    this.unselectBox = function(new_object, keep_lock){

        if (new_object==null){
            if (this.viewManager.mainView && this.viewManager.mainView.transform_control.visible)
            {
                //unselect first time
                this.viewManager.mainView.transform_control.detach();
            }else{
                //unselect second time
                if (this.selected_box){
                    // restore from highlight
                    if (this.selected_box.in_highlight){
                        this.cancelFocus(this.selected_box);    

                        if (!keep_lock){
                            this.view_state.lock_obj_in_highlight = false;
                        }
                    } else{
                        // unselected finally
                        //this.selected_box.material.color = new THREE.Color(parseInt("0x"+get_obj_cfg_by_type(this.selected_box.obj_type).color.slice(1)));
                        //this.selected_box.material.opacity = this.data.cfg.box_opacity;
                        this.boxOp.unhighlightBox(this.selected_box);
                        //this.floatLabelManager.unselect_box(this.selected_box.obj_local_id, this.selected_box.obj_type);
                        this.fastToolBox.hide();

                        if (!keep_lock){
                            this.view_state.lock_obj_track_id = "";
                        }

                        this.imageContextManager.boxes_manager.onBoxUnselected(this.selected_box.obj_local_id, this.selected_box.obj_type);
                        this.selected_box = null;
                        this.boxEditor.detach();

                        this.onSelectedBoxChanged(null);
                    }
                }
                else{
                    // just an empty click
                    return;
                }
            }
        }
        else{
            // selected other box
            //unselect all
            this.viewManager.mainView.transform_control.detach();

            
            if (this.selected_box){
                
                // restore from highlight
                
                if (this.selected_box.in_highlight){
                    this.cancelFocus(this.selected_box); 
                    if (!keep_lock){
                        this.view_state.lock_obj_in_highlight = false;
                    }
                }

                this.selected_box.material.color = new THREE.Color(parseInt("0x"+globalObjectCategory.get_obj_cfg_by_type(this.selected_box.obj_type).color.slice(1)));
                this.selected_box.material.opacity = this.data.cfg.box_opacity;                
                //this.floatLabelManager.unselect_box(this.selected_box.obj_local_id);
                this.fastToolBox.hide();
                this.imageContextManager.boxes_manager.onBoxUnselected(this.selected_box.obj_local_id, this.selected_box.obj_type);

                this.selected_box = null;
                this.boxEditor.detach();
                if (!keep_lock)
                    this.view_state.lock_obj_track_id = "";
            }
        }



        this.render();

    };



    this.selectBox = function(object){

        if (this.selected_box != object){
            // unselect old bbox
            
            var in_highlight = false;

            if (this.selected_box){
                in_highlight = this.selected_box.in_highlight;
                this.unselectBox(this.selected_box);
            }

            // select me, the first time
            this.selected_box = object;

            // switch camera
            if (!this.editorCfg.disableMainImageContext){
                var best_camera = this.imageContextManager.choose_best_camera_for_point(
                    this.selected_box.world.frameInfo.sceneMeta,
                    this.selected_box.position);

                if (best_camera){
                    
                    //var image_changed = this.data.set_active_image(best_camera);

                    // if (image_changed){
                    //     this.editorUi.querySelector("#camera-selector").value=best_camera;
                    //     this.imageContextManager.boxes_manager.display_image();
                    // }

                    this.imageContextManager.setBestCamera(best_camera);
                }
            }

            // highlight box
            // shold change this id if the current selected box changed id.
            this.view_state.lock_obj_track_id = object.obj_track_id;

            //this.floatLabelManager.select_box(this.selected_box.obj_local_id);
            
            this.fastToolBox.setPos(this.floatLabelManager.getLabelEditorPos(this.selected_box.obj_local_id));
            this.fastToolBox.setValue(object.obj_type, object.obj_track_id, object.obj_attr);
            this.fastToolBox.show();

            this.boxOp.highlightBox(this.selected_box);

            if (in_highlight){
                this.focusOnSelectedBox(this.selected_box);
            }
            
            this.save_box_info(object); // this is needed since when a frame is loaded, all box haven't saved anything.
                                        // we could move this to when a frame is loaded.
            this.boxEditor.attachBox(object);
            this.onSelectedBoxChanged(object);

        }
        else {
            //reselect the same box
            if (this.viewManager.mainView.transform_control.visible){
                this.change_transform_control_view();
            }
            else{
                //select me the second time
                //object.add(this.viewManager.mainView.transform_control);
                this.viewManager.mainView.transform_control.attach( object );
            }            
        }

        this.render();

        
    };

    this.adjustContainerSize = function()
    {
        let editorRect = this.editorUi.getBoundingClientRect();
        let headerRect = this.editorUi.querySelector("#header").getBoundingClientRect();

        this.container.style.height = editorRect.height - headerRect.height + "px";
    }


    this.onWindowResize= function() {

        this.adjustContainerSize();
        this.boxEditorManager.onWindowResize();

        // use clientwidth and clientheight to resize container
        // but use scrollwidth/height to place other things.
        if ( this.windowWidth != this.container.clientWidth || this.windowHeight != this.container.clientHeight ) {

            //update_mainview();
            if (this.viewManager.mainView)
                this.viewManager.mainView.onWindowResize();

            if (this.boxEditor)
                this.boxEditor.update("dontrender");

            this.windowWidth = this.container.clientWidth;
            this.windowHeight = this.container.clientHeight;
            this.renderer.setSize( this.windowWidth, this.windowHeight );

            //this.viewManager.updateViewPort();

            // update sideview svg if there exists selected box
            // the following update is called in updateSubviewRangeByWindowResize
            // if (this.selected_box){
            //     this.projectiveViewOps.update_view_handle(this.selected_box);
            // }
        }
        
        this.viewManager.render();
    };

    this.change_transform_control_view= function(){
        if (this.viewManager.mainView.transform_control.mode=="scale"){
            this.viewManager.mainView.transform_control.setMode( "translate" );
            this.viewManager.mainView.transform_control.showY=true;
            this.viewManager.mainView.transform_control.showX=true;
            this.viewManager.mainView.transform_control.showz=true;
        }else if (this.viewManager.mainView.transform_control.mode=="translate"){
            this.viewManager.mainView.transform_control.setMode( "rotate" );
            this.viewManager.mainView.transform_control.showY=false;
            this.viewManager.mainView.transform_control.showX=false;
            this.viewManager.mainView.transform_control.showz=true;
        }else if (this.viewManager.mainView.transform_control.mode=="rotate"){
            this.viewManager.mainView.transform_control.setMode( "scale" );
            this.viewManager.mainView.transform_control.showY=true;
            this.viewManager.mainView.transform_control.showX=true;
            this.viewManager.mainView.transform_control.showz=true;
        }
    };

    this.add_box_on_mouse_pos_by_ref = function(){

        let globalP = this.mouse.get_mouse_location_in_world();
        // trans pos to world local pos
        let pos = this.data.world.scenePosToLidar(globalP);

        let refbox = this.autoAdjust.marked_object.ann;
        pos.z = refbox.psr.position.z;

        let id = refbox.obj_id;

        if (this.autoAdjust.marked_object.frame == this.data.world.frameInfo.frame)
        {
            id = "";
        }

        let box = this.add_box(pos, refbox.psr.scale, refbox.psr.rotation, refbox.obj_type, id, refbox.obj_attr);
        
        return box;
    };

    this.add_box_on_mouse_pos= function(obj_type){
        // todo: move to this.data.world
        let globalP = this.mouse.get_mouse_location_in_world();

        // trans pos to world local pos
        let pos = this.data.world.scenePosToLidar(globalP);

        var rotation = new THREE.Euler(0, 0, this.viewManager.mainView.camera.rotation.z+Math.PI/2, "XYZ");
        rotation = this.data.world.sceneRotToLidar(rotation);

        var obj_cfg = globalObjectCategory.get_obj_cfg_by_type(obj_type);
        var scale = {   
            x: obj_cfg.size[0],
            y: obj_cfg.size[1],
            z: obj_cfg.size[2]
        };

        pos.z = -1.8 + scale.z/2;  // -1.8 is height of lidar

        let id = objIdManager.generateNewUniqueId();

        objIdManager.addObject({
            category: obj_type,
            id: id,
        });

        let box = this.add_box(pos, scale, rotation, obj_type, id);
        
        return box;
    };

    this.add_box= function(pos, scale, rotation, obj_type, obj_track_id, obj_attr){
        let box = this.data.world.annotation.add_box(pos, scale, rotation, obj_type, obj_track_id, obj_attr);

        this.floatLabelManager.add_label(box);
        
        this.imageContextManager.boxes_manager.add_box(box);

        this.selectBox(box);
        return box;
    };

    this.save_box_info= function(box){
        box.last_info = {
            //obj_type: box.obj_type,
            position: {
                x: box.position.x,
                y: box.position.y,
                z: box.position.z,
            },
            rotation: {
                x: box.rotation.x,
                y: box.rotation.y,
                z: box.rotation.z,
            },
            scale: {
                x: box.scale.x,
                y: box.scale.y,
                z: box.scale.z,
            }
        }
    };


    // axix, xyz, action: scale, move, direction, up/down
    this.transform_bbox= function(command){
        if (!this.selected_box)
            return;
        
        if (this.data.world && this.data.world.annotation) {
            this.data.world.annotation.undoManager.takeSnapshot(this.data.world.annotation);
        }

        switch (command){
            case 'x_move_up':
                this.boxOp.translate_box(this.selected_box, 'x', 0.05);
                break;
            case 'x_move_down':
                this.boxOp.translate_box(this.selected_box, 'x', -0.05);
                break;
            case 'x_scale_up':
                this.selected_box.scale.x *= 1.01;    
                break;
            case 'x_scale_down':
                this.selected_box.scale.x /= 1.01;
                break;
            
            case 'y_move_up':
                this.boxOp.translate_box(this.selected_box, 'y', 0.05);
                break;
            case 'y_move_down':        
                this.boxOp.translate_box(this.selected_box, 'y', -0.05);            
                break;
            case 'y_scale_up':
                this.selected_box.scale.y *= 1.01;    
                break;
            case 'y_scale_down':
                this.selected_box.scale.y /= 1.01;
                break;
            
            case 'z_move_up':
                this.selected_box.position.z += 0.05;
                break;
            case 'z_move_down':        
                this.selected_box.position.z -= 0.05;
                break;
            case 'z_scale_up':
                this.selected_box.scale.z *= 1.01;    
                break;
            case 'z_scale_down':
                this.selected_box.scale.z /= 1.01;
                break;
            
            case 'z_rotate_left':
                this.selected_box.rotation.z += 0.01;
                break;
            case 'z_rotate_right':
                this.selected_box.rotation.z -= 0.01;
                break;
            
            case 'z_rotate_reverse':        
                if (this.selected_box.rotation.z > 0){
                    this.selected_box.rotation.z -= Math.PI;
                }else{
                    this.selected_box.rotation.z += Math.PI;
                }    
                break;
            case 'z_rotate_right_angle':
                {
                    const scaleX = this.selected_box.scale.x;
                    this.selected_box.scale.x = this.selected_box.scale.y;
                    this.selected_box.scale.y = scaleX;
                }
                this.selected_box.rotation.z -= Math.PI/2;
                break;
            case 'reset':
                this.selected_box.rotation.x = 0;
                this.selected_box.rotation.y = 0;
                this.selected_box.rotation.z = 0;
                this.selected_box.position.z = 0;
                break;

        }
        this.on_box_changed(this.selected_box);    
        
    };


    // function switch_bbox_type(target_type){
    //     if (!this.selected_box)
    //         return;

    //     if (!target_type){
    //         target_type = get_next_obj_type_name(this.selected_box.obj_type);
    //     }

    //     this.selected_box.obj_type = target_type;
    //     var obj_cfg = get_obj_cfg_by_type(target_type);
    //     this.selected_box.scale.x=obj_cfg.size[0];
    //     this.selected_box.scale.y=obj_cfg.size[1];
    //     this.selected_box.scale.z=obj_cfg.size[2];           

        
    //     this.floatLabelManager.set_object_type(this.selected_box.obj_local_id, this.selected_box.obj_type);
    //     this.floatLabelManager.update_label_editor(this.selected_box.obj_type, this.selected_box.obj_track_id);

        
        
    // }

    

    this.keydown= function( ev ) {

        // if (this.keydownDisabled)
        //     return;

        this.operation_state.key_pressed = true;

        switch ( ev.key) {
            case '+':
            case '=':
                this.data.scale_point_size(1.2);
                this.render();
                break;
            case '-':
                this.data.scale_point_size(0.8);
                this.render();
                break;
            case 'z':
                if (ev.ctrlKey || ev.metaKey) {
                    ev.preventDefault();
                    if (ev.shiftKey) {
                        this.redo();
                    } else {
                        this.undo();
                    }
                    break;
                }
                this.viewManager.mainView.transform_control.showX = ! this.viewManager.mainView.transform_control.showX;
                break;
            case 'y':
                if (ev.ctrlKey || ev.metaKey) {
                    ev.preventDefault();
                    this.redo();
                    break;
                }
                break;
            case '1': 
                this.select_previous_object();
                break;
            case '2':
                this.select_next_object();
                break;
            case '3':
            case 'PageUp':
                this.previous_frame();
                break;
            case 'PageDown':
            case '4':
                this.next_frame();
                break;
            case 'p':
                this.downloadWebglScreenShot();
                break;
            
            // case 'v':
            //     this.change_transform_control_view();
            //     break;
            /*
            case 'm':
            case 'M':
                smart_paste();
                break;
            case 'N':    
            case 'n':
                //add_bbox();
                //header.mark_changed_flag();
                break;        
            case 'B':
            case 'b':
                switch_bbox_type();
                self.header.mark_changed_flag();
                self.on_box_changed(this.selected_box);
                break;
            */
            case 'x': // Y
                this.viewManager.mainView.transform_control.showY = ! this.viewManager.mainView.transform_control.showY;
                break;
            case 'c': // Z
                if (ev.ctrlKey){
                    this.mark_bbox(this.selected_box);
                } else {
                    this.viewManager.mainView.transform_control.showZ = ! this.viewManager.mainView.transform_control.showZ;
                }
                break;            
            case ' ': // Spacebar
                //this.viewManager.mainView.transform_control.enabled = ! this.viewManager.mainView.transform_control.enabled;
                this.playControl.pause_resume_play();
                break;
                
            case '5':            
            case '6':
            case '7':
                this.boxEditor.boxView.views[ev.key-5].cameraHelper.visible = !this.boxEditor.boxView.views[ev.key-5].cameraHelper.visible;
                this.render();
                break;
            
            
            case 's':
                    if (ev.ctrlKey){
                        saveWorldList(this.data.worldList);
                    }
                    else if (this.selected_box)
                    {
                        let v = Math.max(this.editorCfg.moveStep * this.selected_box.scale.x, 0.02);
                        this.boxOp.translate_box(this.selected_box, 'x', -v);
                        this.on_box_changed(this.selected_box);
                    }
                    break;
            case 'w':
                if (this.selected_box){
                    let v = Math.max(this.editorCfg.moveStep * this.selected_box.scale.x, 0.02);
                    this.boxOp.translate_box(this.selected_box, 'x', v);
                    this.on_box_changed(this.selected_box);                
                }
                break;
            case 'a':
                if (this.selected_box){
                    let v = Math.max(this.editorCfg.moveStep * this.selected_box.scale.y, 0.02);
                    this.boxOp.translate_box(this.selected_box, 'y', v);
                    this.on_box_changed(this.selected_box);                
                }
                break;
            case 'd':
                if (this.selected_box){
                    let v = Math.max(this.editorCfg.moveStep * this.selected_box.scale.y, 0.02);
                    this.boxOp.translate_box(this.selected_box, 'y', -v);
                    this.on_box_changed(this.selected_box);                
                }
                break;

            case 'q':
                if (this.selected_box){
                    this.boxOp.rotate_z(this.selected_box, this.editorCfg.rotateStep, false);
                    this.on_box_changed(this.selected_box);
                }
                break;
            case 'e':
                if (this.selected_box){
                    this.boxOp.rotate_z(this.selected_box, -this.editorCfg.rotateStep, false);
                    this.on_box_changed(this.selected_box);
                }
                break;
            case 'r':
                if (this.selected_box){
                    //this.transform_bbox("z_rotate_left");
                    this.boxOp.rotate_z(this.selected_box, this.editorCfg.rotateStep, true);
                    this.on_box_changed(this.selected_box);
                }
                break;
            
            case 'f':
                if (this.selected_box){                
                    //this.transform_bbox("z_rotate_right");                
                    this.boxOp.rotate_z(this.selected_box, -this.editorCfg.rotateStep, true);
                    this.on_box_changed(this.selected_box);
                }
                break;
            case 'g':
                this.transform_bbox("z_rotate_reverse");
                break;
            case 't':
                //this.transform_bbox("reset");
                this.showTrajectory();
                break;
            case 'v':
                this.enterBatchEditMode();
                break;
            case 'd':
            case 'D':
                if (ev.ctrlKey){
                    this.remove_selected_box();
                    this.header.updateModifiedStatus();    
                }
                break;
            case 'Delete':
                this.remove_selected_box();
                this.header.updateModifiedStatus();
                break;
            case 'Escape':
                if (this.selected_box){
                    this.unselectBox(null);
                }
                break;
        }
    };



    this.previous_frame= function(){



        if (!this.data.meta)
            return;

        var scene_meta = this.data.get_current_world_scene_meta();

        var frame_index = this.data.world.frameInfo.frame_index-1;

        if (frame_index < 0){
            console.log("first frame");
            this.infoBox.show("Notice", "This is the first frame");
            return;
        }

        this.load_world(scene_meta.scene, scene_meta.frames[frame_index]);

        

    };

    this.last_frame = function()
    {
        let scene_meta = this.data.get_current_world_scene_meta();
        this.load_world(scene_meta.scene, scene_meta.frames[scene_meta.frames.length-1]);
    };
    this.first_frame = function()
    {
        let scene_meta = this.data.get_current_world_scene_meta();
        this.load_world(scene_meta.scene, scene_meta.frames[0]);
    };

    this.next_frame= function(){    



        if (!this.data.meta)
            return;
            
        var scene_meta = this.data.get_current_world_scene_meta();

        var num_frames = scene_meta.frames.length;

        var frame_index = (this.data.world.frameInfo.frame_index +1);

        if (frame_index >= num_frames){
            console.log("last frame");
            this.infoBox.show("Notice", "This is the last frame");
            return;
        }

        this.load_world(scene_meta.scene, scene_meta.frames[frame_index]);
    };

    this.select_next_object= function(){

        var self=this;
        if (this.data.world.annotation.boxes.length<=0)
            return;

        if (this.selected_box){
            this.operation_state.box_navigate_index = this.data.world.annotation.boxes.findIndex(function(x){
                return self.selected_box == x;
            });
        }
        
        this.operation_state.box_navigate_index += 1;            
        this.operation_state.box_navigate_index %= this.data.world.annotation.boxes.length;    
        
        this.selectBox(this.data.world.annotation.boxes[this.operation_state.box_navigate_index]);

    };

    this.select_previous_object= function(){
        var self=this;
        if (this.data.world.annotation.boxes.length<=0)
            return;

        if (this.selected_box){
            this.operation_state.box_navigate_index = this.data.world.annotation.boxes.findIndex(function(x){
                return self.selected_box == x;
            });
        }
        
        this.operation_state.box_navigate_index += this.data.world.annotation.boxes.length-1;            
        this.operation_state.box_navigate_index %= this.data.world.annotation.boxes.length;    
        
        this.selectBox(this.data.world.annotation.boxes[this.operation_state.box_navigate_index]);
    };

    // this.centerMainView =function(){
    //     let offset = this.data.world.coordinatesOffset;
    //     this.viewManager.mainView.orbit.target.x += offset[0];
    //     this.viewManager.mainView.orbit.target.y += offset[1];
    //     this.viewManager.mainView.orbit.target.z += offset[2];        
    // };

    this.on_load_world_finished= function(world){

        document.title = "SUSTech POINTS-" + world.frameInfo.scene;
        // switch view positoin
        this.moveAxisHelper(world);
        this.moveRangeCircle(world);
        this.lookAtWorld(world);
        this.unselectBox(null, true);
        this.unselectBox(null, true);
        this.render();
        this.imageContextManager.attachWorld(world);
        this.imageContextManager.render_2d_image();
        this.render2dLabels(world);
        this.update_frame_info(world.frameInfo.scene, world.frameInfo.frame);

        this.select_locked_object();
        
        //load_obj_ids_of_scene(world.frameInfo.scene);
        objIdManager.setCurrentScene(world.frameInfo.scene);

        // preload after the first world loaded
        // otherwise the loading of the first world would be too slow
        this.data.preloadScene(world.frameInfo.scene, world);
    };
    this.moveAxisHelper = function(world) {
        world.webglGroup.add(this.axis);
    };

    this.mainViewOffset = [0,0,0];

    this.lookAtWorld = function(world){
        let newOffset = [
                world.coordinatesOffset[0] - this.mainViewOffset[0],
                world.coordinatesOffset[1] - this.mainViewOffset[1],
                world.coordinatesOffset[2] - this.mainViewOffset[2],
            ];
        
        this.mainViewOffset = world.coordinatesOffset;
        
        this.viewManager.mainView.orbit.target.x += newOffset[0];
        this.viewManager.mainView.orbit.target.y += newOffset[1];
        this.viewManager.mainView.orbit.target.z += newOffset[2];

        this.viewManager.mainView.camera.position.x += newOffset[0];
        this.viewManager.mainView.camera.position.y += newOffset[1];
        this.viewManager.mainView.camera.position.z += newOffset[2];

        this.viewManager.mainView.orbit.update();
        
    };

    this.load_world = async function(sceneName, frame, onFinished){

        this.data.dbg.dump();

        logger.log(`load ${sceneName}, ${frame}`);

        var self=this;
        //stop if current world is not ready!
        if (this.data.world && !this.data.world.preloaded()){
            console.error("current world is still loading.");
            return;
        }

        if (this.selected_box && this.selected_box.in_highlight){
            this.cancelFocus(this.selected_box);
        }

        this._clearStackPreview(false);

        if (this.viewManager.mainView && this.viewManager.mainView.transform_control.visible)
        {
            //unselect first time
            this.viewManager.mainView.transform_control.detach();
        }

        var world = await this.data.getWorld(sceneName, frame);

        if (world)
        {
            this.data.activate_world(
                world, 
                function(){
                    self.on_load_world_finished(world);
                    if (onFinished)
                        onFinished();
                    
                }
            );
        }

        
    };


    this.undo = async function() {
        if (await this.restoreLastBatchDelete())
            return;

        if (this.data.world && this.data.world.annotation) {
            var result = this.data.world.annotation.undo();
            if (result) {
                this.imageContextManager.attachWorld(this.data.world);
                this.imageContextManager.render_2d_image();
                this.render2dLabels(this.data.world);
                this.render();
                this.header.updateModifiedStatus();
            }
        }
    };

    this.redo = function() {
        if (this.data.world && this.data.world.annotation) {
            var result = this.data.world.annotation.redo();
            if (result) {
                this.imageContextManager.attachWorld(this.data.world);
                this.imageContextManager.render_2d_image();
                this.render2dLabels(this.data.world);
                this.render();
                this.header.updateModifiedStatus();
            }
        }
    };

    this.remove_box = function(box, render=true){
        if (box === this.selected_box){
            this.unselectBox(null,true);
            this.unselectBox(null,true); //twice to safely unselect.
            this.selected_box = null;
            //this.remove_selected_box();
        } 
        


        this.do_remove_box(box, false); // render later.

        // this should be after do-remove-box
        // subview renderings don't need to be done again after
        // the box is removed.
        if (box.boxEditor)
        {
            if (box.boxEditor){
                box.boxEditor.detach("donthide");
            }
            else{
                console.error("what?");
            }
        }


        this.header.updateModifiedStatus();

        if (render)
            this.render();
        
    };

    this.remove_selected_box= function(){
        this.remove_box(this.selected_box);
    };

    this.do_remove_box = function(box, render=true){

        if (!box.annotator)
            this.restore_box_points_color(box, render);

        this.imageContextManager.boxes_manager.remove_box(box.obj_local_id);

        this.floatLabelManager.remove_box(box);
        this.fastToolBox.hide();
                    
        //this.selected_box.dispose();
        
        box.world.annotation.unload_box(box);
        box.world.annotation.remove_box(box);

        box.world.annotation.setModified();
    },

    this.clear= function(){

        this.header.clear_box_info();
        //this.editorUi.querySelector("#image").innerHTML = '';
        
        this.unselectBox(null);
        this.unselectBox(null);

        this.header.clear_frame_info();

        this.imageContextManager.clear_main_canvas();
        this.boxEditor.detach();


        this.data.world.unload();
        this.data.world= null; //dump it
        this.floatLabelManager.remove_all_labels();
        this.fastToolBox.hide();
        this.render();
    };

    this.update_frame_info= function(scene, frame){
        var self = this;
        this.header.set_frame_info(scene, frame, function(sceneName){
            self.scene_changed(sceneName)});
    };

    //box edited
    this.on_box_changed = function(box){

        if (!this.imageContextManager.hidden())
            this.imageContextManager.boxes_manager.update_box(box);

        this.header.update_box_info(box);
        //floatLabelManager.update_position(box, false);  don't update position, or the ui is annoying.
        
        box.world.annotation.setModified();
        
        

        this.updateBoxPointsColor(box);
        this.save_box_info(box);
        
        

        if (box.boxEditor){
            box.boxEditor.onBoxChanged();
        }
        else{
            console.warn("[StackAnno] on_box_changed skipped boxEditor sync", {
                frame: box.world && box.world.frameInfo ? box.world.frameInfo.frame : null,
                objId: box.obj_track_id || null,
            });
        }

        this.autoAdjust.syncFollowers(box);

        // if (box === this.data.world.radar_box){
        //     this.data.world.move_radar(box);
        // }

        if (box.on_box_changed){
            box.on_box_changed();
        }

        this.header.updateModifiedStatus();
        this.render();
    };

    // box removed, restore points color.
    this.restore_box_points_color= function(box,render=true){
        if (this.data.cfg.color_obj != "no"){
            box.world.lidar.reset_box_points_color(box);
            box.world.lidar.update_points_color();
            if (render)
                this.render();
        }
        
    };

    this.updateBoxPointsColor= function(box){
        if (this.data.cfg.color_obj != "no"){
            if (box.last_info){
                box.world.lidar.set_box_points_color(box.last_info, {x: this.data.cfg.point_brightness, y: this.data.cfg.point_brightness, z: this.data.cfg.point_brightness});
            }

            box.world.lidar.set_box_points_color(box);
            box.world.lidar.update_points_color();            
        }
    };

    this.onSelectedBoxChanged= function(box){

        if (box){        
            this.header.update_box_info(box);
            // this.floatLabelManager.update_position(box, true);
            // this.fastToolBox.setPos(this.floatLabelManager.getLabelEditorPos(box.obj_local_id));
            this.imageContextManager.boxes_manager.onBoxSelected(box.obj_local_id, box.obj_type);


            //this.boxEditor.attachBox(box);

            this.render();
            //this.boxEditor.boxView.render();

            //this.updateSubviewRangeByWindowResize(box);
            
        } else {
            this.header.clear_box_info();
        }

    };

    this.render2dLabels= function(world){
        if (this.editorCfg.disableMainView)
            return;

        this.floatLabelManager.remove_all_labels();
        var self=this;
        world.annotation.boxes.forEach(function(b){
            self.floatLabelManager.add_label(b);
        })

        if (this.selected_box){
            //this.floatLabelManager.select_box(this.selected_box.obj_local_id)
            this.fastToolBox.show();
            this.fastToolBox.setValue(this.selected_box.obj_type, this.selected_box.obj_track_id, this.selected_box.obj_attr);
        }
    };

    this.add_global_obj_type= function(){

        var self = this;
        var sheet = window.document.styleSheets[1];

        let obj_type_map = globalObjectCategory.obj_type_map;

        for (var o in obj_type_map){
            var rule = '.'+o+ '{color:'+obj_type_map[o].color+';'+ 
                                'stroke:' +obj_type_map[o].color+ ';'+
                                'fill:' +obj_type_map[o].color+ '22' + ';'+
                                '}';
            sheet.insertRule(rule, sheet.cssRules.length);
        }

        function color_str(v){
            let c =  Math.round(v*255);
            if (c < 16)
                return "0" + c.toString(16);
            else
                return c.toString(16);
        }

        for (let idx=0; idx<=32; idx++){
            let c = globalObjectCategory.get_color_by_id(idx);
            let color = "#" + color_str(c.x) + color_str(c.y) + color_str(c.z);

            var rule = '.color-'+idx+ '{color:'+color+';'+ 
                                'stroke:' +color+ ';'+
                                'fill:' +color+ '22' + ';'+
                                '}';
            sheet.insertRule(rule, sheet.cssRules.length);
        }

        // obj type selector
        var options = "";
        for (var o in obj_type_map){
            options += '<option value="'+o+'" class="' +o+ '">'+o+ '</option>';        
        }

        this.editorUi.querySelector("#floating-things #object-category-selector").innerHTML = options;
        //this.editorUi.querySelector("#batch-editor-tools-wrapper #object-category-selector").innerHTML = options;

        // submenu of new
        var items = "";
        for (var o in obj_type_map){
            items += '<div class="menu-item cm-new-item ' + o + '" id="cm-new-'+o+'" uservalue="' +o+ '"><div class="menu-item-text">'+o+ '</div></div>';        
        }

        this.editorUi.querySelector("#new-submenu").innerHTML = items;

        this.contextMenu.installMenu("newSubMenu", this.editorUi.querySelector("#new-submenu"), (event)=>{
            let obj_type = event.currentTarget.getAttribute("uservalue");
            let box = self.add_box_on_mouse_pos(obj_type);
            //switch_bbox_type(event.currentTarget.getAttribute("uservalue"));
            //self.boxOp.grow_box(box, 0.2, {x:2, y:2, z:3});
            //self.auto_shrink_box(box);
            //self.on_box_changed(box);

            let noscaling = event.shiftKey;

            self.boxOp.auto_rotate_xyz(box, null, null, function(b){
                self.on_box_changed(b);
            }, noscaling);
            return true;
        });

        // // install click actions
        // for (var o in obj_type_map){        
        //     this.editorUi.querySelector("#cm-new-"+o).onclick = (event)=>{

        //         // hide context men
        //         // let context menu object handle this.
        //         // this.editorUi.querySelector("#context-menu-wrapper").style.display="none";

        //         // process event
        //         var obj_type = event.currentTarget.getAttribute("uservalue");
        //         let box = self.add_box_on_mouse_pos(obj_type);
        //         //switch_bbox_type(event.currentTarget.getAttribute("uservalue"));
        //         //self.boxOp.grow_box(box, 0.2, {x:2, y:2, z:3});
        //         //self.auto_shrink_box(box);
        //         //self.on_box_changed(box);

        //         self.boxOp.auto_rotate_xyz(box, null, null, function(b){
        //             self.on_box_changed(b);
        //         });
                
        //     }
        // }

    };

    this.interpolate_selected_object= function(){

        let scene = this.data.world.frameInfo.scene; 
        let frame = this.data.world.frameInfo.frame;
        let obj_id = this.selected_box.obj_track_id;

        this.boxOp.interpolate_selected_object(scene, obj_id, frame, (s,fs)=>{
            this.onAnnotationUpdatedByOthers(s, fs);
        });

        
    };

    this.onAnnotationUpdatedByOthers = function(scene, frames){
        return this.data.onAnnotationUpdatedByOthers(scene, frames);
    }

    this.openStackAnnotationPanel = function() {
        if (!this.selected_box) {
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }
        if (!this.selected_box.obj_track_id) {
            this.infoBox.show("Error", "Please assign object track ID first.");
            return;
        }

        const scene = this.data.world.frameInfo.scene;
        const frame = this.data.world.frameInfo.frame;
        const objId = this.selected_box.obj_track_id;

        const self = this;
        fetch("/propagate_init?scene=" + encodeURIComponent(scene) +
              "&frame=" + encodeURIComponent(frame) +
              "&obj_id=" + encodeURIComponent(objId))
            .then(r => r.json())
            .then(data => {
                if (data.error) {
                    self.infoBox.show("Error", data.error);
                    return;
                }
                self._showPropagationPanel(data);
            })
            .catch(err => {
                self.infoBox.show("Error", "Failed to init stack annotation: " + err.message);
            });
    };

    this.openPropagationPanel = function() {
        this.openStackAnnotationPanel();
    };

    this._showPropagationPanel = function(initData) {
        const panel = document.getElementById("propagation-panel");
        if (!panel) return;
        const self = this;

        const sp = this.spatialPropagation;
        sp.init(
            initData.scene,
            initData.anchor_frame,
            initData.obj_id,
            initData.box_psr,
            initData.reference_count,
            initData.frame_ids,
            initData.current_idx,
            initData.obj_type,
            initData.obj_attr,
            this.stackReference ? this.stackReference.frame : initData.anchor_frame,
            this.stackReference ? this.stackReference.psr : initData.box_psr,
            this.stackReference ? this.stackReference.frameRadius : 4,
            this.stackReference ? this.stackReference.framesUsed : null
        );

        document.getElementById("propagation-obj-info").textContent =
            initData.scene + " | obj:" + initData.obj_id +
            " | ref:" + initData.reference_count + "pts | frame:" + initData.anchor_frame;

        this._updatePropagationProgress(initData.current_idx, initData.total_frames, initData.anchor_frame);

        document.getElementById("propagation-method").textContent = "";
        document.getElementById("propagation-point-count").textContent = "";
        document.getElementById("propagation-stack-radius").value = String(
            this.stackReference ? this.stackReference.frameRadius : 4
        );
        self._showStackStatus("");
        self._clearStackPreview(false);
        self._refreshStackReferenceStatus();

        document.getElementById("propagation-controls").style.display = "flex";

        panel.style.display = "block";

        const closeBtn = document.getElementById("propagation-close");
        closeBtn.onclick = function() { self._closePropagationPanel(); };

        document.getElementById("propagation-btn-forward").onclick = function() {
            self._startPropagation("forward");
        };
        document.getElementById("propagation-btn-backward").onclick = function() {
            self._startPropagation("backward");
        };
        document.getElementById("propagation-btn-stop").onclick = function() {
            self._stopPropagation();
        };
        document.getElementById("propagation-btn-stack-show").onclick = function() {
            self._showStackPreview();
        };
        document.getElementById("propagation-btn-stack-lock").onclick = function() {
            self._lockStackReference();
        };
        document.getElementById("propagation-btn-stack-clear").onclick = function() {
            self._clearStackPreview(true);
        };

        self._makePanelDraggable(panel);
    };

    this._getBoxPsrFromBox = function(box) {
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
    };

    this._showStackStatus = function(message, isError=false) {
        const status = document.getElementById("propagation-stack-status");
        if (!status) {
            return;
        }
        status.textContent = message || "";
        status.style.color = isError ? "#fca5a5" : "#93c5fd";
    };

    this._clearStackPreview = function(clearStatus=true) {
        if (this.stackPreviewMesh) {
            if (this.stackPreviewMesh.parent) {
                this.stackPreviewMesh.parent.remove(this.stackPreviewMesh);
            }
            if (this.stackPreviewMesh.geometry) {
                this.stackPreviewMesh.geometry.dispose();
            }
            if (this.stackPreviewMesh.material) {
                this.stackPreviewMesh.material.dispose();
            }
            this.stackPreviewMesh = null;
        }
        if (clearStatus) {
            this._showStackStatus("");
        }
        this.render();
    };

    this._refreshStackReferenceStatus = function() {
        const status = document.getElementById("propagation-reference-status");
        if (!status) {
            return;
        }

        if (this.stackReference && this.stackReference.frame) {
            const usedCount = this.stackReference.framesUsed ? this.stackReference.framesUsed.length : null;
            status.textContent =
                "Locked reference: frame " + this.stackReference.frame +
                " | fixed size + yaw refine enabled | " +
                (usedCount ? ("apply " + usedCount + " stacked frames") : ("radius ±" + this.stackReference.frameRadius));
            status.style.color = "#a7f3d0";
            return;
        }

        if (this.data.world && this.data.world.frameInfo) {
            status.textContent =
                "No locked reference yet. Current frame " +
                this.data.world.frameInfo.frame +
                " will be used as a temporary reference if you apply now.";
            status.style.color = "#fde68a";
            return;
        }

        status.textContent = "";
    };

    this._lockStackReference = function() {
        if (!this.selected_box || !this.data.world || !this.data.world.frameInfo) {
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }

        const radiusInput = document.getElementById("propagation-stack-radius");
        let frameRadius = parseInt(radiusInput ? radiusInput.value : "4", 10);
        if (isNaN(frameRadius) || frameRadius < 0) {
            frameRadius = 4;
        }

        const psr = this._getBoxPsrFromBox(this.selected_box);
        const frame = this.data.world.frameInfo.frame;
        const previewMatchesCurrent =
            this.lastStackPreviewScene === this.data.world.frameInfo.scene &&
            this.lastStackPreviewFrame === frame &&
            this.lastStackPreviewRadius === frameRadius;
        this.stackReference = {
            frame: frame,
            psr: psr,
            frameRadius: frameRadius,
            framesUsed: (previewMatchesCurrent && this.lastStackPreviewFrames) ? Array.from(this.lastStackPreviewFrames) : null,
        };
        console.log("[StackAnno] lock-reference " + JSON.stringify({
            scene: this.data.world.frameInfo.scene,
            frame: frame,
            objId: this.selected_box ? this.selected_box.obj_track_id : null,
            frameRadius: frameRadius,
            previewMatchesCurrent: previewMatchesCurrent,
            framesUsed: this.stackReference.framesUsed,
            referencePsr: {
                position: {
                    x: Number(psr.position.x.toFixed(3)),
                    y: Number(psr.position.y.toFixed(3)),
                    z: Number(psr.position.z.toFixed(3)),
                },
                rotation: {
                    x: Number(psr.rotation.x.toFixed(3)),
                    y: Number(psr.rotation.y.toFixed(3)),
                    z: Number(psr.rotation.z.toFixed(3)),
                },
                scale: {
                    x: Number(psr.scale.x.toFixed(3)),
                    y: Number(psr.scale.y.toFixed(3)),
                    z: Number(psr.scale.z.toFixed(3)),
                },
            },
        }));
        this.spatialPropagation.setFixedReference(frame, psr, frameRadius, this.stackReference.framesUsed);
        this._showStackStatus("Locked stack reference on frame " + frame + ", yaw refinement is enabled during propagation.");
        this._refreshStackReferenceStatus();
    };

    this._showStackPreview = async function() {
        if (!this.selected_box) {
            this.infoBox.show("Error", "Please select a box first.");
            return;
        }
        if (!this.data.world || !this.data.world.frameInfo) {
            this.infoBox.show("Error", "Current frame is not ready.");
            return;
        }

        const radiusInput = document.getElementById("propagation-stack-radius");
        let frameRadius = parseInt(radiusInput ? radiusInput.value : "4", 10);
        if (isNaN(frameRadius) || frameRadius < 0) {
            frameRadius = 4;
            if (radiusInput) {
                radiusInput.value = "4";
            }
        }

        const scene = this.data.world.frameInfo.scene;
        const frame = this.data.world.frameInfo.frame;
        const objId = this.selected_box.obj_track_id || "";
        const boxPsr = this._getBoxPsrFromBox(this.selected_box);

        this._showStackStatus("Loading stacked points...");

        try {
            const params = new URLSearchParams({
                scene: scene,
                frame: frame,
                obj_id: objId,
                box_psr: JSON.stringify(boxPsr),
                frame_radius: String(frameRadius),
                margin: "1.2",
                max_points: "30000",
            });
            const resp = await fetch("/stack_object_points?" + params.toString());
            const data = await resp.json();
            if (data.error) {
                this._showStackStatus(data.error, true);
                return;
            }

            const points = data.points || [];
            if (!points.length) {
                this._showStackStatus("No stacked points returned.", true);
                return;
            }
            this.lastStackPreviewFrames = Array.isArray(data.frames_used) ? data.frames_used.map(f => String(f)) : [];
            this.lastStackPreviewScene = scene;
            this.lastStackPreviewFrame = frame;
            this.lastStackPreviewRadius = frameRadius;
            console.log("[StackAnno] preview-stack " + JSON.stringify({
                scene: scene,
                frame: frame,
                objId: objId,
                frameRadius: frameRadius,
                framesUsed: data.frames_used || [],
                pointCount: data.point_count || points.length,
                rawPointCount: data.raw_point_count || points.length,
            }));

            this._clearStackPreview(false);

            const positions = new Float32Array(points.length * 3);
            const colors = new Float32Array(points.length * 3);
            for (let i = 0; i < points.length; i++) {
                positions[i * 3] = points[i][0];
                positions[i * 3 + 1] = points[i][1];
                positions[i * 3 + 2] = points[i][2];
                colors[i * 3] = 0.30;
                colors[i * 3 + 1] = 0.85;
                colors[i * 3 + 2] = 1.00;
            }

            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
            geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
            geometry.computeBoundingSphere();

            const material = new THREE.PointsMaterial({
                size: Math.max(this.data.cfg.point_size * 1.6, 2.0),
                vertexColors: true,
                transparent: true,
                opacity: 0.85,
                sizeAttenuation: false,
            });

            this.stackPreviewMesh = new THREE.Points(geometry, material);
            this.stackPreviewMesh.name = "stack-preview";
            this.data.world.webglGroup.add(this.stackPreviewMesh);
            this.render();

            const usedFrames = (data.frames_used || []).length;
            this._showStackStatus(
                "Stacked " + usedFrames + " frames, " +
                (data.point_count || points.length) + " pts (" +
                (data.raw_point_count || points.length) + " raw)"
            );
        } catch (err) {
            this._showStackStatus("Failed to stack points: " + err.message, true);
        }
    };

    this._makePanelDraggable = function(panel) {
        const header = document.getElementById("propagation-panel-header");
        if (!header) return;

        let isDragging = false;
        let startX, startY, startLeft, startTop;

        header.style.cursor = "move";

        header.onmousedown = function(e) {
            if (e.target.id === "propagation-close") return;
            isDragging = true;
            startX = e.clientX;
            startY = e.clientY;
            const rect = panel.getBoundingClientRect();
            startLeft = rect.left;
            startTop = rect.top;
            panel.style.transition = "none";
            document.body.style.userSelect = "none";
            e.preventDefault();
        };

        document.addEventListener("mousemove", function(e) {
            if (!isDragging) return;
            const dx = e.clientX - startX;
            const dy = e.clientY - startY;
            panel.style.left = (startLeft + dx) + "px";
            panel.style.top = (startTop + dy) + "px";
            panel.style.transform = "none";
        });

        document.addEventListener("mouseup", function() {
            if (!isDragging) return;
            isDragging = false;
            document.body.style.userSelect = "";
        });
    };

    this._startPropagation = function(direction) {
        const self = this;
        const sp = this.spatialPropagation;
        const radiusInput = document.getElementById("propagation-stack-radius");
        let frameRadius = parseInt(radiusInput ? radiusInput.value : "4", 10);
        if (isNaN(frameRadius) || frameRadius < 0) {
            frameRadius = 4;
        }

        if (this.stackReference) {
            sp.setFixedReference(this.stackReference.frame, this.stackReference.psr, frameRadius, this.stackReference.framesUsed);
            this._showStackStatus("Using locked stack reference from frame " + this.stackReference.frame + ".");
        } else if (this.selected_box && this.data.world && this.data.world.frameInfo) {
            const currentPsr = this._getBoxPsrFromBox(this.selected_box);
            sp.setFixedReference(this.data.world.frameInfo.frame, currentPsr, frameRadius);
            this._showStackStatus("Using current frame as temporary fixed reference.");
        }
        this._refreshStackReferenceStatus();
        console.log("[StackAnno] apply-start " + JSON.stringify({
            scene: this.data.world && this.data.world.frameInfo ? this.data.world.frameInfo.scene : null,
            startFrame: this.data.world && this.data.world.frameInfo ? this.data.world.frameInfo.frame : null,
            objId: this.selected_box ? this.selected_box.obj_track_id : null,
            direction: direction,
            frameRadius: frameRadius,
            lockedReference: this.stackReference ? {
                frame: this.stackReference.frame,
                framesUsed: this.stackReference.framesUsed || null,
            } : null,
        }));

        document.getElementById("propagation-controls").style.display = "none";

        sp.start(direction,
            function(stepResult) {
                return self._onPropagationStep(stepResult);
            },
            function() {
                self._onPropagationStop();
            }
        );
    };

    this._onPropagationStep = function(stepResult) {
        const self = this;
        const sp = this.spatialPropagation;
        const progress = sp.getProgress();
        const scene = sp.state.scene;
        const tgtFrame = stepResult.frame;

        return new Promise(function(resolve) {
            self.load_world(scene, tgtFrame, function() {
                Promise.resolve(
                    sp.applyBoxToCurrentFrame(stepResult.psr, sp.state.objType, sp.state.objAttr)
                ).then((applyResult) => {
                    const box = applyResult && applyResult.box
                        ? applyResult.box
                        : self.data.world.annotation.findBoxByTrackId(sp.state.objId);
                    if (box) {
                        self.selectBox(box);
                        self.focusOnSelectedBox(box);
                    }

                    self._updatePropagationProgress(progress.current, progress.total, tgtFrame);

                    document.getElementById("propagation-method").textContent =
                        stepResult.method ? ("mode: " + stepResult.method) : "";
                    document.getElementById("propagation-point-count").textContent =
                        "pts: " + (stepResult.pointCount !== null ? stepResult.pointCount : "?");

                    resolve(applyResult || null);
                }).catch((err) => {
                    self.infoBox.show("Error", "Failed to apply stacked box: " + err.message);
                    resolve();
                });
            });
        });
    };

    this._switchToFrameLight = async function(sceneName, frame, onFinished) {
        var self = this;

        if (this.data.world && !this.data.world.preloaded()) {
            console.error("current world is still loading.");
            return;
        }

        if (this.selected_box && this.selected_box.in_highlight) {
            this.cancelFocus(this.selected_box);
        }

        if (this.viewManager.mainView && this.viewManager.mainView.transform_control.visible) {
            this.viewManager.mainView.transform_control.detach();
        }

        var world = await this.data.getWorld(sceneName, frame);

        if (world) {
            this.data.activate_world(world, function() {
                document.title = "SUSTech POINTS-" + world.frameInfo.scene;
                self.moveAxisHelper(world);
                self.moveRangeCircle(world);
                self.update_frame_info(world.frameInfo.scene, world.frameInfo.frame);
                objIdManager.setCurrentScene(world.frameInfo.scene);

                if (world.lidar.points) {
                    self.render();
                    if (onFinished) onFinished();
                } else {
                    var retries = 0;
                    var maxRetries = 50;
                    var checkReady = function() {
                        if (world.lidar.points) {
                            self.render();
                            if (onFinished) onFinished();
                        } else if (retries < maxRetries) {
                            retries++;
                            setTimeout(checkReady, 100);
                        } else {
                            self.render();
                            if (onFinished) onFinished();
                        }
                    };
                    setTimeout(checkReady, 100);
                }
            });
        }
    };

    this._stopPropagation = function() {
        this.spatialPropagation.stop();
        this._closePropagationPanel();
    };

    this._onPropagationStop = function() {
        document.getElementById("propagation-controls").style.display = "flex";
        document.getElementById("propagation-method").textContent = "done";
    };

    this._updatePropagationProgress = function(current, total, frame) {
        document.getElementById("propagation-frame-label").textContent =
            "Frame: " + frame + " (" + (current + 1) + "/" + total + ")";
        const pct = total > 0 ? ((current + 1) / total * 100) : 0;
        document.getElementById("propagation-progress-fill").style.width = pct + "%";
    };

    this._closePropagationPanel = function() {
        this.spatialPropagation.stop();
        this._clearStackPreview(true);
        this._refreshStackReferenceStatus();
        const panel = document.getElementById("propagation-panel");
        if (panel) panel.style.display = "none";
    };

    this.init(editorUi);

};

export{Editor}
