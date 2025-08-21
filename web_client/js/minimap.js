/**
 * @file <client/js/minimap.js>
 * 
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 * 
 * MIT License
 * Copyright (c) 2021 NRL Lab
 */

var canvas;

var entities = new Map();

var OFFSET = 0;
var SCALE = 0;

/* ----------------------- */

var InitializeMinimap = function (minimap_panel) {
  var _width = minimap_panel.width() * 0.98;
  var _height = minimap_panel.height() * 0.98;

  OFFSET = _height / 2
  SCALE = _height / 6

  canvas = new fabric.StaticCanvas('mini-map',
    {
      // backgroundColor: 'rgb(225,225,225)',
      width: _width,
      height: _height,
      centeredRotation: true,
      centeredScaling: true
      // containerClassName: 'theCanvas',
      // preserveObjectStacking: true,
    });
}

function update_minimap() {

  /* Update entities in the minimap */
  for (const key in sceneEntities) {
    if (sceneEntities.hasOwnProperty(key)) {
      const entity = sceneEntities[key].entity;

      if(entity.type == 'box') {

        if(key in entities) {

          /* Update position */
          entities[key].left = OFFSET + SCALE * entity.position.x;
          entities[key].top = OFFSET - SCALE * entity.position.y;

        } else {

          /* Add box to map */
          var box = new fabric.Rect({
            left: OFFSET + SCALE * entity.position.x,
            top: OFFSET - SCALE * entity.position.y,
            fill: '#333',
            width: SCALE * entity.scale.x,
            height: SCALE * entity.scale.y,
            originX: "center",
            originY: "center"
          });

          box.rotate(2 * Math.acos(entity.orientation.w) * 180 / Math.PI)
          
          entities[key] = box;
          canvas.add(box);
        }

      } else if(entity.type == 'e-puck_leader') {
        // TODO: Number of followers

        if(key in entities) {

          /* Update position */
          entities[key].left = OFFSET + SCALE * entity.position.x;
          entities[key].top = OFFSET - SCALE * entity.position.y;

        } else {

          /* Add leader to map */
          var robot = new fabric.Circle({
            radius: SCALE * 0.035,
            fill: 'red',
            left: OFFSET + SCALE * entity.position.x,
            top: OFFSET - SCALE * entity.position.y,
            originX: "center",
            originY: "center",
            // strokeWidth: 15,
            // stroke: 'rgba(100,200,200,0.5)'
          });

          entities[key] = robot;
          canvas.add(robot);
        }

      } else if(entity.type == 'e-puck') {

        // console.log(entity);

        /* Display information about workers that are connectors */
        if(entity.user_data.state == "C") {

          if(key in entities) {

            /* Update position */
            entities[key].left = OFFSET + SCALE * entity.position.x;
            entities[key].top = OFFSET - SCALE * entity.position.y;
  
          } else {
  
            /* Add connector to map */
            var robot = new fabric.Circle({
              radius: SCALE * 0.035,
              fill: 'blue',
              left: OFFSET + SCALE * entity.position.x,
              top: OFFSET - SCALE * entity.position.y,
              originX: "center",
              originY: "center",
              // strokeWidth: 15,
              // stroke: 'rgba(100,200,200,0.5)'
            });
  
            robot.rotate(2 * Math.acos(entity.orientation.w) * 180 / Math.PI)
            
            entities[key] = robot;
            canvas.add(robot);
          }
        }

        // TODO: Delete connector from minimap when they become a follower again

      } else if(entity.type == 'rectangle_task') {
        // TODO: 
        // - Remaining demand
        // - Total demand
        // - Number of robots needed

        if(key in entities) {

          /* Update position */
          entities[key].left = OFFSET + SCALE * entity.position.x;
          entities[key].top = OFFSET - SCALE * entity.position.y;

        } else {

          /* Add task to map */
          var task = new fabric.Rect({
            left: OFFSET + SCALE * entity.position.x,
            top: OFFSET - SCALE * entity.position.y,
            fill: 'rgba(200,0,0,0.5)',
            width: SCALE * entity.scale.x,
            height: SCALE * entity.scale.y,
            originX: "center",
            originY: "center"
          });

          task.rotate(2 * Math.acos(entity.orientation.w) * 180 / Math.PI)
          
          entities[key] = task;
          canvas.add(task);
        }
      }
    }
  }

  // console.log(sceneEntities);

  canvas.renderAll()
}