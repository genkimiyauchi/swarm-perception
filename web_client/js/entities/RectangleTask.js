/**
 * @file <client/js/entities/RectangleTask.js>
 * 
 * @author Genki Miyauchi - <g.miyauchi@sheffield.ac.uk>
 * 
 */

 class RectangleTask {
    constructor(entity, scale, EntityLoadingFinishedFn) {
        this.scale = scale;
        this.entity = entity;

        var that = this;
        var geometry = new THREE.BoxBufferGeometry(
            entity.scale.x * scale,
            entity.scale.y * scale,
            entity.scale.z * scale,
        );

        /* Bring above ground */
        geometry.translate(0, 0, entity.scale.z * scale / 2 - 0.1);

        var demand = entity.task.demand;

        var color = null;
        if (demand == 0) {
            color = 0x00ff00;
        } else {
            color = 0xff0000;
        }

        var material = new THREE.MeshPhongMaterial({
            color: color,
            transparent: true,
            opacity: 0.2,
        });

        this.task = new THREE.Mesh(geometry, material);

        this.task.rotation.setFromQuaternion(new THREE.Quaternion(
            entity.orientation.x,
            entity.orientation.y,
            entity.orientation.z,
            entity.orientation.w));

        var meshParent = new THREE.Group();
        /* Add all parts to a parent mesh */
        meshParent.add(this.task);

        /* Add task outline */
        const edges = new THREE.EdgesGeometry( geometry );
        const line = new THREE.LineSegments( edges, new THREE.LineBasicMaterial( { color: 0x000000 } ) );
        meshParent.add(line);

        if(window.mode == Mode.DEBUG) {
            /* Amount of task completed in percentage */
            var task_completed = Math.floor((1 - entity.task.demand / entity.task.init_demand) * 100);

            this.sprite = new THREE.TextSprite({
                alignment: 'center',
                color: '#000000',
                fontFamily: '"Times New Roman", Times, serif',
                fontSize: 8,
                text: [
                    task_completed + "%",
                ].join('\n'),
            });

            this.sprite.position.z = entity.scale.z * scale + 4;

            meshParent.add(this.sprite);
        }

        /* Update mesh parent */
        meshParent.position.x = entity.position.x * scale;
        meshParent.position.y = entity.position.y * scale;
        meshParent.position.z = entity.position.z * scale;

        that.mesh = meshParent;

        EntityLoadingFinishedFn(that);
    }

    getMesh() {
        return this.mesh;
    }

    update(entity) {
        this.mesh.position.x = entity.position.x * this.scale;
        this.mesh.position.y = entity.position.y * this.scale;

        this.mesh.rotation.setFromQuaternion(new THREE.Quaternion(
            entity.orientation.x,
            entity.orientation.y,
            entity.orientation.z,
            entity.orientation.w));

        if(this.mesh) {

            if(window.mode == Mode.DEBUG) {
                /* Update task demand */
                var task_completed = Math.floor((1 - entity.task.demand / entity.task.init_demand) * 100);

                this.sprite.text = [
                    task_completed + "%",
                ].join('\n');
            }

            /* Update color of task */
            var color = null;
            if (entity.task.demand == 0) {
                color = 0x00ff00;
            } else {
                color = 0xff0000;
            }
    
            this.mesh.children[0].material.color.setHex(color);
        }
    }
}