class URDF_viewer extends HTMLElement {

    //
    get angles() {
        const angles = {}
        this.robots.forEach(r => {
            for (let name in r.urdf.joints) angles[name] = r.urdf.joints[name].urdf.angle
        })
        return angles
    }
    set angles(val) { this._setAngles(val) }

    // Cached mesh loaders
    // static get STLLoader() {
    //     this._stlloader = this._stlloader || new THREE.STLLoader()
    //     return this._stlloader
    // }

    // static get DAELoader() {
    //     this._daeloader = this._daeloader || new THREE.ColladaLoader()
    //     return this._daeloader
    // }

    // static get TextureLoader() {
    //     this._textureloader = this._textureloader || new THREE.TextureLoader()
    //     return this._textureloader
    // }

    /* Utilities */
    // forEach and filter function wrappers because 
    // HTMLCollection does not them by default
    static forEach(coll, func) { return [].forEach.call(coll, func) }
    static filter(coll, func) { return [].filter.call(coll, func) }

    //Sleep function
    static sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }

    // take a vector "x y z" and process it into
    // an array [x, y, z]
    static _processTuple(val) {
        if (!val) return [0, 0, 0]
        return val.trim().split(/\s+/g).map(num => parseFloat(num))
    }

    // applies a rotation a threejs object in URDF order
    static _applyRotation(obj, rpy) {
        obj.rotateOnAxis(new THREE.Vector3(0, 0, 1), rpy[2])
        obj.rotateOnAxis(new THREE.Vector3(0, 1, 0), rpy[1])
        obj.rotateOnAxis(new THREE.Vector3(1, 0, 0), rpy[0])
    }


    /* Public API */
    static init(object) {
        const rnd = document.getElementById('renderer')

        this.scene = new THREE.Scene();
        this.scene.background = new THREE.Color(0xf0f0f0);

        // const ambientLight = new THREE.AmbientLight(this.ambientColor)
        // this.scene.add(ambientLight)

        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 100);
        this.camera.position.set(2, 2, 2);
        this.scene.add(this.camera);

        // var light = new THREE.SpotLight( 0xffffff, 1.5 );
        // light.position.set( 0, 3, 3 );
        // light.castShadow = true;
        // light.shadow = new THREE.LightShadow( this.camera );
        // light.shadow.bias = -0.000222;
        // light.shadow.mapSize.width = 1024;
        // light.shadow.mapSize.height = 1024;
        // this.scene.add( light );

        this.scene.add(new THREE.HemisphereLight(0x443333, 0x111122));

        this.addShadowedLight(1, 1, 1, 0xffffff, 1.35);
        this.addShadowedLight(-1, -1, -1, 0xffffff, 1.35);

        this.renderer = new THREE.WebGLRenderer();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        this.renderer.setPixelRatio(window.devicePixelRatio);
        // this.renderer.gammaInput = true;
        // this.renderer.gammaOutput = true;

        this.renderer.shadowMap.enabled = true;
        rnd.appendChild(this.renderer.domElement);

        //this.renderer.setClearColor(0xffffff, 1);

        //World setup
        // const world = new THREE.Object3D()
        // this.scene.add(world)

        var geometry = new THREE.PlaneBufferGeometry(10, 10);
        geometry.rotateX(- Math.PI / 2);
        var material = new THREE.ShadowMaterial({ opacity: 0.2 });
        var plane = new THREE.Mesh(geometry, material);
        plane.position.y = 0;
        plane.receiveShadow = true;
        this.scene.add(plane);

        var helper = new THREE.GridHelper(10, 100);
        helper.position.y = 0;
        helper.material.opacity = 0.25;
        helper.material.transparent = true;
        this.scene.add(helper);

        var worldAxis = new THREE.AxesHelper(20);
        this.scene.add(worldAxis);

        for (let key in object.urdf.jointMap) {
            //console.log(self.jointMap[key])
            const joint = object.urdf.jointMap[key]
            if (joint.urdf.type !== 'fixed')
                this.scene.add(joint.axes)
        }

        // var geometry = new THREE.BoxGeometry( 100, 100, 100 );
        // var material = new THREE.MeshBasicMaterial( { color: 0x00ff00 } );
        // this.cube = new THREE.Mesh( geometry, material );
        // this.scene.add( this.cube );

        this.scene.add(object);

        this.orbitControls = new THREE.OrbitControls(this.camera, this.renderer.domElement);

        this.transformControls = new THREE.TransformControls(this.camera, this.renderer.domElement);
        // this.transformControls.addEventListener( 'change', render );

        this.transformControls.attach(object)
        this.scene.add(this.transformControls)
    }

    static animate() {

        // Read more about requestAnimationFrame at http://www.paulirish.com/2011/requestanimationframe-for-smart-animating/
        requestAnimationFrame(this.animate.bind(this));
        // this.cube.rotation.x += 0.1;
        // this.cube.rotation.y += 0.1;


        // Render the this.scene.
        this.renderer.render(this.scene, this.camera);
        this.orbitControls.update();
        this.transformControls.update();

    }

    static addShadowedLight(x, y, z, color, intensity) {

        var directionalLight = new THREE.DirectionalLight(color, intensity);
        directionalLight.position.set(x, y, z);
        this.scene.add(directionalLight);

        directionalLight.castShadow = true;

        var d = 1;
        directionalLight.shadow.camera.left = -d;
        directionalLight.shadow.camera.right = d;
        directionalLight.shadow.camera.top = d;
        directionalLight.shadow.camera.bottom = -d;

        directionalLight.shadow.camera.near = 1;
        directionalLight.shadow.camera.far = 4;

        directionalLight.shadow.mapSize.width = 1024;
        directionalLight.shadow.mapSize.height = 1024;

        directionalLight.shadow.bias = -0.002;

    }

    // static defaultMeshLoader(path, ext, done) {

    //     if (/\.stl$/i.test(path)){
    //         this.STLLoader.load(path, function(geom) {
    //             console.log('stl loder')
    //             const mesh = new THREE.Mesh()
    //             mesh.geometry = geom
    //             console.log(mesh)
    //             done(mesh)
    //         })
    //     }  
    //     else if (/\.dae$/i.test(path))
    //         this.DAELoader.load(path, dae => {

    //             console.log('dae')
    //             done(dae.scene)
    //         })
    //     else
    //         console.warn(`Could note load model at ${path}:\nNo loader available`)
    // }

    static addModule(reader) {
        const parser = new DOMParser()
        const urdf = parser.parseFromString(reader.result, 'text/xml')

        console.log(this.jointNumber)

        // r=this.robots[0]
        // var lastchild = this.filter(r.children, n => n.nodeName.length > 0 )[0]

        // this.robots[0].urdf.joints

        let endWithLink = false
        let endWithJoint = false

        const links = Object.keys(this.linkMap)
        const joints = Object.keys(this.jointMap)

        if (links.length > joints.length) {
            endWithLink = true
            endWithJoint = false
        } else {
            endWithLink = false
            endWithJoint = true
        }

        console.log('Ending with a joint?:', endWithJoint)

        this.forEach(urdf.children, r => {
            // const obj = new THREE.Object3D()
            // obj.name = r.getAttribute('name')
            // obj.urdf = { node: r }


            this.forEach(r.children, n => {
                const type = n.nodeName.toLowerCase()

                if (endWithJoint) {
                    if (type === 'link') {
                        console.log('link_' + this.jointNumber)
                        const newlink_name = 'link_' + this.jointNumber
                        this.linkMap[newlink_name] = this._processLink(n)
                        this.linkMap[newlink_name].name = newlink_name

                        //add link as child of last joint
                        const lastjoint_name = joints[joints.length - 1]
                        const parentname = this.jointMap[lastjoint_name].name
                        this.jointMap[parentname].add(this.linkMap[newlink_name])

                        //update joint and link maps
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap
                        //console.log(this.robots[0])
                    }
                    else if (type === 'joint') {
                        console.warn('Cannot attach two consecutive joints')
                        return
                    }
                } else if (endWithLink) {
                    if (type === 'link') {
                        //Note: if two consecutive links are attached together a fixed joint must be created first to connect the two
                        //console.log("link_bis")
                        //add fixed joint
                        //
                        const lastjoint_name = joints[joints.length - 1]
                        const newjoint_name = lastjoint_name + '_bis'
                        this.jointMap[newjoint_name] = this._createFixedJoint(newjoint_name)

                        //add joint as child of last link                        
                        const lastlink_name = links[links.length - 1]
                        const parentname = this.linkMap[lastlink_name].name
                        this.linkMap[parentname].add(this.jointMap[newjoint_name])

                        //add link
                        //
                        const newlink_name = lastlink_name + '_bis'
                        console.log(newlink_name)
                        this.linkMap[newlink_name] = this._processLink(n)
                        this.linkMap[newlink_name].name = newlink_name

                        //add link as child of last joint
                        this.jointMap[newjoint_name].add(this.linkMap[newlink_name])

                        //update joint and link maps
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap
                    }
                    else if (type === 'joint') {
                        console.log('joint_' + (this.jointNumber+1))
                        const newjoint_name = 'joint_' + (this.jointNumber+1)
                        this.jointMap[newjoint_name] = this._processJoint(n)
                        this.jointMap[newjoint_name].name = newjoint_name

                        //add joint as child of last link                        
                        const lastlink_name = links[links.length - 1]
                        const parentname = this.linkMap[lastlink_name].name
                        this.linkMap[parentname].add(this.jointMap[newjoint_name])

                        //add joint object to the robot objects and update joint and link maps
                        //this.robots[0].add(this.jointMap[newjoint_name])
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap

                        //this.jointMap[newjoint_name]

                        var URDF_change_eH = new URDF_change_eventHandler();

                        self = this
                        URDF_change_eH.addEventListener('urdf-change', function (event) {

                            alert(event.message);

                            const joint = self.jointMap[newjoint_name]
                            self._createSlider(joint)

                        });

                        URDF_change_eH.start();

                    }
                }

            })


        })



    }

    static addModuleYAML(doc, URDF_string) {
        const parser = new DOMParser()
        const urdf = parser.parseFromString(URDF_string, 'text/xml')

        
        console.log(this.jointNumber)

        // r=this.robots[0]
        // var lastchild = this.filter(r.children, n => n.nodeName.length > 0 )[0]

        // this.robots[0].urdf.joints

        let endWithLink = false
        let endWithJoint = false

        var links = Object.keys(this.linkMap)
        var joints = Object.keys(this.jointMap)

        if (links.length > joints.length) {
            endWithLink = true
            endWithJoint = false
        } else {
            endWithLink = false
            endWithJoint = true
        }

        console.log('Ending with a joint?:', endWithJoint)

        this.forEach(urdf.children, r => {
            // const obj = new THREE.Object3D()
            // obj.name = r.getAttribute('name')
            // obj.urdf = { node: r }


            this.forEach(r.children, n => {
                const type = n.nodeName.toLowerCase()

                if (endWithJoint) {
                    if (type === 'link') {
                        console.log('link_' + this.jointNumber)
                        const newlink_name = 'link_' + this.jointNumber
                        this.linkMap[newlink_name] = this._processLink(n)
                        this.linkMap[newlink_name].name = newlink_name

                        //add link as child of last joint
                        const lastjoint_name = joints[joints.length - 1]
                        const parentname = this.jointMap[lastjoint_name].name
                        this.jointMap[parentname].add(this.linkMap[newlink_name])

                        //update joint and link maps
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap
                        //console.log(this.robots[0])
                    }
                    else if (type === 'joint') {
                        console.warn('Cannot attach two consecutive joints')
                        return
                    }
                } else if (endWithLink) {
                    if (type === 'link') {
                        //Note: if two consecutive links are attached together a fixed joint must be created first to connect the two
                        //console.log("link_bis")
                        //add fixed joint
                        //
                        const lastjoint_name = joints[joints.length - 1]
                        const newjoint_name = lastjoint_name + '_bis'
                        this.jointMap[newjoint_name] = this._createFixedJoint(newjoint_name)

                        //add joint as child of last link                        
                        const lastlink_name = links[links.length - 1]
                        const parentname = this.linkMap[lastlink_name].name
                        this.linkMap[parentname].add(this.jointMap[newjoint_name])

                        //add link
                        //
                        const newlink_name = lastlink_name + '_bis'
                        console.log(newlink_name)
                        this.linkMap[newlink_name] = this._processLink(n)
                        this.linkMap[newlink_name].name = newlink_name

                        //add link as child of last joint
                        this.jointMap[newjoint_name].add(this.linkMap[newlink_name])

                        //update joint and link maps
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap

                        links = Object.keys(this.linkMap)
                        joints = Object.keys(this.jointMap)
                    }
                    else if (type === 'joint') {
                        console.log('joint_' + (this.jointNumber+1))
                        const newjoint_name = 'joint_' + (this.jointNumber+1)
                        this.jointMap[newjoint_name] = this._processJoint(n)
                        this.jointMap[newjoint_name].name = newjoint_name

                        //add joint as child of last link                        
                        const lastlink_name = links[links.length - 1]
                        const parentname = this.linkMap[lastlink_name].name
                        this.linkMap[parentname].add(this.jointMap[newjoint_name])

                        //add joint object to the robot objects and update joint and link maps
                        //this.robots[0].add(this.jointMap[newjoint_name])
                        this.robots[0].urdf.joints = this.jointMap
                        this.robots[0].urdf.links = this.linkMap

                        links = Object.keys(this.linkMap)
                        joints = Object.keys(this.jointMap)

                        //this.jointMap[newjoint_name]

                        var URDF_change_eH = new URDF_change_eventHandler();

                        self = this
                        URDF_change_eH.addEventListener('urdf-change', function (event) {

                            alert(event.message);

                            const joint = self.jointMap[newjoint_name]
                            self._createSlider(joint)

                        });

                        URDF_change_eH.start();

                    }
                }

                this.lastModKin = doc.kinematics;
                //console.log("save parameters")
            })


        })

       

        return this.robots
    }

    static showURDF(reader) {
        // THREE.Object3D.DefaultUp = new THREE.Vector3(0, 0, 1)

        //const controlsel = document.getElementById('controls')
        // const sliderList = document.querySelector('#controls ul')
        // const moduleselector = document.getElementById('selector')

        this.lastModKin = {
            joint: { proximal: { a_pl: 0, alpha_pl: 0, p_pl: 0, n_pl: 0, delta_pl: 0}, 
            distal: { a_dl: 0, alpha_dl: 0, p_dl: 0, n_dl: 0.4, delta_dl: 0},
            joint: { delta_j: 0, type: 'rotational'}
            }, 
            link: { a_l: 0, alpha_l: 0, p_l: 0, n_l: 0, delta_l_in: 0, delta_l_out: 0}
        }

        const parser = new DOMParser()
        const urdf = parser.parseFromString(reader.result, 'text/xml')

        this.robots = []
        this.jointNumber = 0

        console.log(urdf.children)
        this.forEach(urdf.children, r => {

            const links = []
            const joints = []
            const materials = []
            const obj = new THREE.Object3D()
            obj.name = r.getAttribute('name')
            obj.urdf = { node: r }
            console.log(r.children)

            // Process the <joint> and <link> nodes
            this.forEach(r.children, n => {
                const type = n.nodeName.toLowerCase()
                if (type === 'link') links.push(n)
                else if (type === 'joint') joints.push(n)
                else if (type === 'material') materials.push(n)
            })

            console.log(links)
            console.log(joints)
            console.log(materials)

            // Create the <material> map
            this.materialMap = {}
            this.forEach(materials, m => {
                const name = m.getAttribute('name')
                this.materialMap[name] = this._processMaterial(m)
            })

            console.log(this.materialMap)

            // Create the <link> map
            this.linkMap = {}
            this.forEach(links, l => {
                const name = l.getAttribute('name')
                this.linkMap[name] = this._processLink(l)
            })

            console.log(this.linkMap)

            // // Create the <joint> map
            this.jointMap = {}
            this.forEach(joints, j => {
                const name = j.getAttribute('name')
                this.jointMap[name] = this._processJoint(j)
            })

            console.log(this.jointMap)

            for (let key in this.linkMap) this.linkMap[key].parent ? null : obj.add(this.linkMap[key])

            obj.urdf.joints = this.jointMap
            obj.urdf.links = this.linkMap

            //console.log(obj)

            const rpy_ros2three = [-Math.PI / 2, -Math.PI / 2, 0]
            //console.log(rpy_ros2three)
            this._applyRotation(obj, rpy_ros2three)
            //object.rotateOnAxis ( new THREE.Vector3(1, 0, 0), - Math.PI / 2 )

            this.init(obj);
            this.animate();

            this.robots.push(obj)
        })
        //let event = new Event ()
        //URDF_viewer.prototype.dispatchEvent(new CustomEvent('urdf-processed', { bubbles: true, cancelable: true, composed: true }))

        var URDF_processed_eH = new URDF_processed_eventHandler();

        var self = this;
        URDF_processed_eH.addEventListener('urdf-processed', function (event) {

            alert(event.message);

            //console.log(self.jointMap)
            for (let key in self.jointMap) {
                //console.log(self.jointMap[key])
                const joint = self.jointMap[key]
                if (joint.urdf.type !== 'fixed')
                    self._createSlider(joint)
            }

        });

        URDF_processed_eH.start();

        console.log(this.robots[0])
        console.log(Object.keys(this.linkMap))

        // var rdr = new FileReader();
        // rdr.readAsText(new File('/home/edoardo/catkin_ws/src/modular/urdf/module_elbow.yaml'));  

        // var doc = jsyaml.load(rdr.result);
        // console.log(doc)

        return this.robots
    }

    static setAngle(jointname, angle) {
        this.robots.forEach(r => {
            const joint = r.urdf.joints[jointname]
            if (joint) joint.urdf.setAngle(angle)
        })
        this._dirty = true
    }

    static setAngles(angles) {
        for (name in angles) this.setAngle(name, angles[name])
    }

    /* Private Functions */
    static _createSlider(joint) {
        const sliderList = document.querySelector('#controls ul')

        const li = document.createElement('li')
        li.innerHTML =
            `
        <span title="${joint.name}">${joint.name}</span>
        <input type="range" value="0" step="0.01" class="range"/>
        <input type="number" step="0.01" class="number"/>
        `
        //li.setAttribute('joint-type', joint.urdf.type)

        //sliderList = document.querySelector('#controls ul')
        sliderList.appendChild(li)

        const slider = li.querySelector('input[type="range"]')
        const input = li.querySelector('input[type="number"]')
        li.update = () => {
            //let val = joint.urdf.type === 'revolute' ? joint.urdf.angle * Math.RAD2DEG : joint.urdf.angle
            // if (Math.abs(val) > 1) val = val.toFixed(1)
            // else val = val.toPrecision(2)
            let val = joint.urdf.angle.toPrecision(3)

            input.value = parseFloat(val)
            slider.value = joint.urdf.angle

            // if (viewer.ignoreLimits) {
            //     slider.min = -6.28
            //     slider.max = 6.28

            //     input.min = -6.28
            //     input.max = 6.28
            // } else {
            slider.min = joint.urdf.limits.lower
            slider.max = joint.urdf.limits.upper

            input.min = joint.urdf.limits.lower
            input.max = joint.urdf.limits.upper
            // }
        }

        switch (joint.urdf.type) {
            case 'continuous':
            case 'prismatic':
            case 'revolute':
                break;
            default:
                li.update = () => { }
                input.remove()
                slider.remove()
        }

        slider.addEventListener('input', () => {
            this.setAngle(joint.name, slider.value)
            li.update()
        })

        input.addEventListener('change', () => {
            this.setAngle(joint.name, input.value)
            li.update()
        })

        li.update()

    }

    static _processMaterial(m) {
        const name = m.getAttribute('name')
        const material = new THREE.MeshLambertMaterial()
        material.name = name
        material.urdf = { node: m }

        this.forEach(m.children, c => {

            if (c.nodeName.toLowerCase() === 'color') {
                let rgba = c.getAttribute('rgba')
                    .split(/\s/g)
                    .map(v => parseFloat(v))

                material.color.r = rgba[0]
                material.color.g = rgba[1]
                material.color.b = rgba[2]
                material.opacity = rgba[3]

                if (material.opacity < 1) material.transparent = true
            }
        })

        return material
    }

    static _createFixedJoint(joint_name) {
        const jointType = "fixed"
        const joint_obj = new THREE.Object3D()
        joint_obj.name = joint_name
        
        const parser = new DOMParser()
        const fixed_joint_urdf ='<joint type="fixed"><axis rpy="0 0 0" xyz="0 0 1"/><origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.0"/><limit lower="-3.14" upper="3.14"/></joint>'
        const urdf_node = parser.parseFromString(fixed_joint_urdf, 'text/xml')
        //console.log(urdf_node.children[0])

        joint_obj.urdf = {
            node: urdf_node.children[0], type: jointType, angle: 0, axis: null,
            limits: { lower: 0, upper: 0 },
            ignoreLimits: false,
        }
        
        console.log(this.lastModKin.joint.proximal.n_pl + this.lastModKin.joint.distal.n_dl)
        let xyz = [0.0, this.lastModKin.joint.proximal.p_pl + this.lastModKin.joint.distal.p_dl, this.lastModKin.joint.proximal.n_pl + this.lastModKin.joint.distal.n_dl ] //this.lastModKin.p_pl + this.lastModKin.p_dl, this.lastModKin.n_pl + this.lastModKin.n_dl
        let rpy = [this.lastModKin.joint.distal.alpha_dl, 0.0, 0.0] //this.lastModKin.alpha_dl


        this._applyRotation(joint_obj, rpy)
        joint_obj.position.set(xyz[0], xyz[1], xyz[2])

        return joint_obj
    }

    static _processJoint(j) {
        const jointType = j.getAttribute('type')
        const joint_obj = new THREE.Object3D()
        joint_obj.name = j.getAttribute('name')
        joint_obj.urdf = {
            node: j, type: jointType, angle: 0, axis: null,
            limits: { lower: 0, upper: 0 },
            ignoreLimits: false,
            setAngle: () => { }
        }

        let parent = null
        let child = null
        let xyz = [0, 0, 0]
        let rpy = [0, 0, 0]

        // Extract the attributes
        this.forEach(j.children, n => {
            const type = n.nodeName.toLowerCase()
            if (type === 'origin') {
                xyz = this._processTuple(n.getAttribute('xyz'))
                rpy = this._processTuple(n.getAttribute('rpy'))
            } else if (type === 'child') {
                child = this.linkMap[n.getAttribute('link')]
            } else if (type === 'parent') {
                parent = this.linkMap[n.getAttribute('link')]
            } else if (type === 'limit') {
                joint_obj.urdf.limits.lower = parseFloat(n.getAttribute('lower') || joint_obj.urdf.limits.lower)
                joint_obj.urdf.limits.upper = parseFloat(n.getAttribute('upper') || joint_obj.urdf.limits.upper)
            }
        })

        console.log(this.lastModKin.joint.proximal.n_pl + this.lastModKin.joint.distal.n_dl)
        if (!joint_obj.name) {
            xyz = [0.0, this.lastModKin.joint.proximal.p_pl + this.lastModKin.joint.distal.p_dl, this.lastModKin.joint.proximal.n_pl + this.lastModKin.joint.distal.n_dl ] //this.lastModKin.p_pl + this.lastModKin.p_dl, this.lastModKin.n_pl + this.lastModKin.n_dl
            rpy = [this.lastModKin.joint.distal.alpha_dl, 0.0, 0.0] //this.lastModKin.alpha_dl
        }

        // const axes = new THREE.AxesHelper( 1 );
        // axes.position.set(xyz[0], xyz[1], xyz[2]);
        // axes.rotation.set(rpy[0], rpy[1], rpy[2]);
        // joint_obj.add(axes)

        var a = new THREE.Vector3(1, 0, 0);
        var b = new THREE.Vector3(0, 1, 0);
        var c = new THREE.Vector3(0, 0, 1);
        var origin = new THREE.Vector3(0, 0, 0);
        var length = 0.3;
        var hex_a = 0x3511d8;
        var hex_b = 0xd81111;
        var hex_c = 0x1ee81e;
        var arrow_a = new THREE.ArrowHelper(a, origin, length, hex_a);
        var arrow_b = new THREE.ArrowHelper(b, origin, length, hex_b);
        var arrow_c = new THREE.ArrowHelper(c, origin, length, hex_c);
        var refFrame = new THREE.Group();
        refFrame.add(arrow_a);
        refFrame.add(arrow_b);
        refFrame.add(arrow_c);
        //refFrame.position.set(xyz[0], xyz[1], xyz[2]);
        //refFrame.rotation.set(rpy[0], rpy[1], rpy[2]);
        joint_obj.add(refFrame);


        // Join the links

        //The IF might be removed!!!
        if (parent != null && child != null) {
            parent.add(joint_obj)
            joint_obj.add(child)
        }

        this._applyRotation(joint_obj, rpy)
        joint_obj.position.set(xyz[0], xyz[1], xyz[2])

        // Set up the rotate function
        const origRot = new THREE.Quaternion().copy(joint_obj.quaternion)
        const origPos = new THREE.Vector3().copy(joint_obj.position)
        const axisnode = this.filter(j.children, n => n.nodeName.toLowerCase() === 'axis')[0]

        if (axisnode) {
            const axisxyz = axisnode.getAttribute('xyz').split(/\s+/g).map(num => parseFloat(num))
            joint_obj.urdf.axis = new THREE.Vector3(axisxyz[0], axisxyz[1], axisxyz[2])
            joint_obj.urdf.axis.normalize()
        }

        switch (jointType) {
            case 'fixed': break;
            case 'continuous':
                joint_obj.urdf.limits.lower = -Infinity
                joint_obj.urdf.limits.upper = Infinity

            // fall through to revolute joint 'setAngle' function
            case 'revolute':
                joint_obj.urdf.setAngle = function (angle = null) {
                    if (!joint_obj.urdf.axis) return
                    if (angle == null) return

                    if (!joint_obj.urdf.ignoreLimits) {
                        angle = Math.min(joint_obj.urdf.limits.upper, angle)
                        angle = Math.max(joint_obj.urdf.limits.lower, angle)
                    }

                    // FromAxisAngle seems to rotate the opposite of the
                    // expected angle for URDF, so negate it here
                    const delta = new THREE.Quaternion().setFromAxisAngle(joint_obj.urdf.axis, angle)
                    joint_obj.quaternion.multiplyQuaternions(origRot, delta)

                    joint_obj.urdf.angle = angle
                }

                this.jointNumber++

                break

            case 'prismatic':
                joint_obj.urdf.setAngle = function (angle = null) {
                    if (!joint_obj.urdf.axis) return
                    if (angle == null) return

                    if (!joint_obj.urdf.ignoreLimits) {
                        angle = Math.min(joint_obj.urdf.limits.upper, angle)
                        angle = Math.max(joint_obj.urdf.limits.lower, angle)
                    }

                    joint_obj.position.copy(origPos);
                    joint_obj.position.addScaledVector(joint_obj.urdf.axis, angle)

                    joint_obj.urdf.angle = angle

                    this.jointNumber++
                }
                break

            case 'floating':
            case 'planar':
                // TODO: Support these joint types
                console.warn(`'${jointType}' joint not yet supported`)
        }

        // copy the 'setAngle' function over to 'set' so
        // it makes sense for other joint types (prismatic, planar)
        // TODO: Remove the 'setAngle' function
        // TODO: Figure out how to handle setting and getting angles of other types
        joint_obj.urdf.set = joint_obj.urdf.setAngle

        return joint_obj
    }

    static _processLink(l) {
        const visualNodes = this.filter(l.children, n => n.nodeName.toLowerCase() === 'visual')
        const link_obj = new THREE.Object3D()
        link_obj.name = l.getAttribute('name')
        link_obj.urdf = { node: l }
        //console.log(link_obj)

        this.forEach(visualNodes, vn => this._processVisualNode(vn, link_obj))

        return link_obj
    }

    static _processVisualNode(vn, link_obj) {
        let xyz = [0, 0, 0]
        let rpy = [0, 0, 0]
        let size = [0, 0, 0]
        let scale = [1, 1, 1]
        let mesh = null

        const material = new THREE.MeshLambertMaterial()
        this.forEach(vn.children, n => {
            const type = n.nodeName.toLowerCase()
            if (type === 'material') {
                //if the material is one of those stored in materialMap copy it from there
                for (var key in this.materialMap) {
                    var value = this.materialMap[key];
                    //console.log(value.name)
                    if (n.getAttribute('name') === value.name) {
                        //console.log(value)
                        material.copy(value)

                    }
                }

                //otherwise  check if the material properties are specified directly
                this.forEach(n.children, c => {
                    if (c.nodeName.toLowerCase() === 'color') {
                        let rgba = c.getAttribute('rgba')
                            .split(/\s/g)
                            .map(v => parseFloat(v))

                        material.color.r = rgba[0]
                        material.color.g = rgba[1]
                        material.color.b = rgba[2]
                        material.opacity = rgba[3]

                        if (material.opacity < 1) material.transparent = true
                    }
                    // } else if (c.nodeName.toLowerCase() === 'texture') {
                    //     const filename = c.getAttribute('filename').replace(/^(package:\/\/)/, '')
                    //     const path = pkg + '/' + filename
                    //     material.map = this._textureloader.load(path)
                    // }
                })
            } else if (type === 'geometry') {
                const geoType = n.children[0].nodeName.toLowerCase()
                if (geoType === 'box') {
                    requestAnimationFrame(() => {
                        const mesh = new THREE.Mesh()
                        //mesh.up = new THREE.Vector3(0,0,1)
                        //mesh.DefaultUp = new THREE.Vector3(0, 0, 1)
                        mesh.geometry = new THREE.BoxGeometry(1, 1, 1)
                        mesh.material = material


                        if (!n.children[0].getAttribute('size')) {
                            size = [1, 1, 1]
                        } else {
                            size = n.children[0].getAttribute('size').trim().split(/\s+/g).map(num => parseFloat(num))
                        }
                        //console.log(xyz)
                        link_obj.add(mesh)

                        //this._applyRotation(mesh, rpy)
                        mesh.rotation.set(rpy[0], rpy[1], rpy[2])
                        mesh.position.set(xyz[0], xyz[1], xyz[2])
                        mesh.scale.set(size[0], size[1], size[2])
                    })
                } else if (geoType === 'sphere') {
                    requestAnimationFrame(() => {
                        const mesh = new THREE.Mesh()
                        mesh.geometry = new THREE.SphereGeometry(1, 20, 20)
                        mesh.material = material

                        const radius = parseFloat(n.children[0].getAttribute('radius')) || 0
                        mesh.position.set(xyz[0], xyz[1], xyz[2])
                        mesh.scale.set(radius, radius, radius)
                    })
                } else if (geoType === 'cylinder') {
                    requestAnimationFrame(() => {
                        const radius = parseFloat(n.children[0].getAttribute('radius')) || 0
                        const length = parseFloat(n.children[0].getAttribute('length')) || 0

                        const mesh = new THREE.Mesh()
                        //mesh.up = new THREE.Vector3(0,0,1)
                        mesh.geometry = new THREE.CylinderBufferGeometry(1, 1, 1, 25)
                        mesh.material = material
                        mesh.scale.set(radius, length, radius)

                        const cyl_obj = new THREE.Object3D()
                        cyl_obj.add(mesh)
                        mesh.rotation.set(Math.PI / 2, 0, 0)

                        //console.log(xyz)
                        link_obj.add(cyl_obj)
                        this._applyRotation(cyl_obj, rpy)
                        cyl_obj.position.set(xyz[0], xyz[1], xyz[2])
                    })
                } else if (geoType === 'mesh') {
                    const filename = n.children[0].getAttribute('filename').replace(/^((package:\/\/)|(model:\/\/))/, '')
                    const path = './models' + '/' + filename
                    console.log(path)
                    const ext = path.match(/.*\.([A-Z0-9]+)$/i).pop() || ''
                    let scale_exist = n.children[0].getAttribute('scale')
                    if (scale_exist) scale = this._processTuple(scale_exist)



                    if (/\.stl$/i.test(path)) {
                        //console.log('stl loader')
                        var Loader = new THREE.STLLoader();
                        //console.log(Loader)
                        //console.log(path)
                        Loader.load(path, function (geom) {

                            const mesh = new THREE.Mesh()
                            mesh.geometry = geom
                            //console.log(mesh)
                            if (mesh) {
                                // var meshMaterial = material;
                                // if (geometry.hasColors) {
                                //     meshMaterial = new THREE.MeshPhongMaterial({ opacity: geometry.alpha, vertexColors: THREE.VertexColors });
                                //     mesh.material = meshMaterial
                                // }

                                if (mesh instanceof THREE.Mesh) {
                                    mesh.material = material
                                }

                                link_obj.add(mesh)

                                mesh.position.set(xyz[0], xyz[1], xyz[2])
                                mesh.rotation.set(rpy[0], rpy[1], rpy[2])

                                mesh.scale.set(scale[0], scale[1], scale[2])

                                //this._applyRotation(mesh, [0,0,0])
                                //console.log("1")
                                //console.log(rpy)
                                // const delta = new THREE.Quaternion().setFromAxisAngle(joint_obj.urdf.axis, angle)
                                // joint_obj.quaternion.multiplyQuaternions(origRot, delta)

                                // mesh.castShadow = true;
                                // mesh.receiveShadow = true;

                                //console.log(mesh)
                            }
                        })
                    }
                    // else if (/\.dae$/i.test(path))
                    //     this.DAELoader.load(path, dae => {

                    //         console.log('dae')
                    //         done(dae.scene)
                    //     })
                    else
                        console.warn(`Could note load model at ${path}:\nNo loader available`)

                    //console.log('MESH')
                    // this.defaultMeshLoader(path, ext, obj => {
                    //     if (obj) {
                    //         if (obj instanceof THREE.Mesh) {
                    //             obj.material = material
                    //         }

                    //         link_obj.add(obj)

                    //         obj.position.set(xyz[0], xyz[1], xyz[2])
                    //         obj.rotation.set(0,0,0)
                    //         //obj.scale.set(scale[0], scale[1], scale[2])
                    //         this._applyRotation(obj, rpy)
                    //         console.log(obj)
                    //     }
                    // })                    
                }
            } else if (type === 'origin') {

                xyz = this._processTuple(n.getAttribute('xyz'))
                rpy = this._processTuple(n.getAttribute('rpy'))
                // if(rpy[0] > Math.PI)
                //     rpy[0] = rpy[0] - 2*Math.PI
                // if(rpy[1] > Math.PI)
                //     rpy[1] = rpy[1] - 2*Math.PI
                // if(rpy[2] > Math.PI)
                //     rpy[2] = rpy[2] - 2*Math.PI

            }
        })
    }

}

//Event handler for urdf processed
var URDF_processed_eventHandler = function () {

    this.start = function () {

        this.dispatchEvent({ type: 'urdf-processed', message: 'urdf has been processed!' });
    };
};

Object.assign(URDF_processed_eventHandler.prototype, THREE.EventDispatcher.prototype);

//Event handler for urdf change
var URDF_change_eventHandler = function () {

    this.start = function () {

        this.dispatchEvent({ type: 'urdf-change', message: 'urdf has been changed!' });
    };
};

Object.assign(URDF_change_eventHandler.prototype, THREE.EventDispatcher.prototype);



// URDF_viewer.prototype.dispatchEvent = function (evt) {
//     var listeners = this._getListeners(evt.type, false).slice();
//     for (var i = 0; i < listeners.length; i++)
//         listeners[i].call(this, evt);
//     return !evt.defaultPrevented;
// };

window.URDF_viewer = URDF_viewer