/**
 * @file <client/js/entities/Box.js>
 * 
 * @author Prajankya Sonar - <prajankya@gmail.com>
 * 
 * @project ARGoS3-Webviz <https://github.com/NESTlab/argos3-webviz>
 * 
 * MIT License
 * Copyright (c) 2020 NEST Lab
 */

class Box {
    constructor(entity, scale, EntityLoadingFinishedFn) {
        this.scale = scale;
        this.entity = entity;

        var that = this;
        var geometry = new THREE.BoxBufferGeometry(
            entity.scale.x * scale,
            entity.scale.y * scale,
            entity.scale.z * scale
        );

        /* Bring above ground */
        geometry.translate(0, 0, entity.scale.z * scale * 0.5);

        var color = null;
        if (entity.is_movable) {
            color = 0xff0000;
        } else {
            color = 0x766e64;
        }

        var material = new THREE.MeshPhongMaterial({
            color: color,
        });

        var box = new THREE.Mesh(geometry, material);

        var meshParent = new THREE.Group();
        /* Add all parts to a parent mesh */
        meshParent.add(box);

        const edges = new THREE.EdgesGeometry( geometry );
        const line = new THREE.LineSegments( edges, new THREE.LineBasicMaterial( { color: 0x000000 } ) );
        meshParent.add(line);

        meshParent.rotation.setFromQuaternion(new THREE.Quaternion(
            entity.orientation.x,
            entity.orientation.y,
            entity.orientation.z,
            entity.orientation.w));
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
        if (entity.is_movable) {
            try {
                this.mesh.position.x = entity.position.x * this.scale;
                this.mesh.position.y = entity.position.y * this.scale;

                this.mesh.rotation.setFromQuaternion(new THREE.Quaternion(
                    entity.orientation.x,
                    entity.orientation.y,
                    entity.orientation.z,
                    entity.orientation.w));
            } catch (ignored) { }
        }
    }
}