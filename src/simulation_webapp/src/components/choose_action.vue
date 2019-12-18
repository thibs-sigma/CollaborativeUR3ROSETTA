<template>
  <div class="hello">
    <v-container grid-list-xl justify-center>
      <h1>UR3 simulación - Interfaz de control > Eligir acción</h1>
      <p>Control de las funcionalidades de la demonstración con V-REP.</p>
      <br />

      <!-- BEGIN LINE 1 -->
      <v-layout align-center justify-center row wrap>
        <!-- MENU TILE 1 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(1)"
          @click.stop="dialogDemo = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/demo_gears.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">FULL DEMO</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>

        <v-dialog v-model="dialogDemo" persistent width="1200">
                  <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >DEMOSTRACIÓN</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!nextDemo"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'DEMO' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else> <!-- Second message -->
                <!-- <v-icon size="100" color="black">playlist_add_check</v-icon> -->
                  <img :src="require('../assets/img/demo_gears.png')" style="height: 150px;" />
                <br />
                <span class="headline font-weight-bold">
                  Ahora, la demo va a iniciar<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  TO DO
                </span>
                <br />
              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!nextDemo" color="orange" text dark @click="nextDemo = true"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialogDemo = false;
                  nextDemo = false;
                "
                >Listo!</v-btn
              >
            </v-card-actions>
          </v-card>
        </v-dialog>
        
        <!-- END MENU TILE 1 -->

        <!-- MENU TILE 2 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(2)"
          @click.stop="dialogManip = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/get_object.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">MANIP. PIEZA</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        
        <v-dialog v-model="dialogManip" persistent width="1200">
                  <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >MANIPULACIÓN DE PIEZA</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!nextManip"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'MANIP. PIEZA' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else-if="!nextManip2"> <!-- Second message -->
                <!-- <v-icon size="100" color="black">playlist_add_check</v-icon> -->
                <img :src="require('../assets/img/loading.gif')" style="height: 50px;" />
                <br />
                <span class="headline font-weight-bold">
                  Ahora, la manipulación de piezas va a iniciar<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  Verifica las informaciones en la salida consola en el Terminal. <br/>
                  Una vez que los robots empezan a mover, presiona <b>'Siguiente'</b>
                </span>
                <br />
              </v-flex>

              <v-flex xs12 v-else> <!-- Third message -->

                <!-- <v-icon size="100" color="black">playlist_add_check</v-icon> -->
                <img :src="require('../assets/img/get_object.png')" style="height: 50px;" />
                <br />
                <span class="headline font-weight-bold">
                  Informaciones tiempo-real sobre la simulación<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  Simpre verifica las informaciones completas en la salida consola en el Terminal. <br/>
                </span>
                <br />
                      <!-- BEGIN LINE 1 -->
              <v-layout align-center justify-center row > 
                <v-card height="100%" width="300" hover ripple class="mx-3 my-2 disable-events" to="choose_action">
                  <v-container grid-list-md text-xs-center>
                    <v-layout align-center justify-center >
                        <v-flex xs12>
                          <p>
                            <img :src="require('../assets/img/joint_states.png')" style="height: 100px; margin-top: 0px; margin-bottom: -30px;" />
                          </p>
                          <br />
                          <span class="headline font-weight-bold">JOINT STATES</span>
                          <br />
                          <span class="subheading">
                            <v-data-table :headers="headers" :items="jointStatesTable" hide-actions>
                              <template v-slot:items="props">
                                <td >{{ props.item.topic }}</td>
                                <td class="text-xs-left">{{ props.item.message_received }} {{ props.item.unidad }}</td>
                              </template>
                            </v-data-table>
                          </span>
                          <br />

                        </v-flex>
                    </v-layout>
                    


                  </v-container>
                </v-card>

                <v-card height="100%" width="300" hover ripple class="mx-3 my-2 disable-events" to="choose_action">
                  <v-container grid-list-md text-xs-center>
                    <v-layout align-center justify-center>
                        <v-flex xs12>
                          <p>
                            <img :src="require('../assets/img/gripper.png')" style="height: 100px; margin-top: 0px; margin-bottom: -30px;" />
                          </p>
                          <br />
                          <span class="headline font-weight-bold">GRIPPER</span>
                          <br />
                          <!-- <span class="subheading">
                            Test 3
                          </span>-->
                          <br />
                        </v-flex>
                    </v-layout>
                  </v-container>
                </v-card>

                <v-card height="100%" width="300" hover ripple class="mx-3 my-2 disable-events" to="choose_action">
                  <v-container grid-list-md text-xs-center>
                    <v-layout align-center justify-center>
                        <v-flex xs12>
                          <p>
                            <img :src="require('../assets/img/moveit.png')" style="height: 50px; margin-top: 50px; margin-bottom: -30px;" />
                          </p>
                          <br />
                          <span class="headline font-weight-bold">MOTION PLANNING</span>
                          <br />
                          <!-- <span class="subheading">
                            Test 3
                          </span>-->
                          <br />
                        </v-flex>
                    </v-layout>
                  </v-container>
                </v-card>

               </v-layout>

              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!nextManip" color="orange" text dark @click="nextManip = true;"
                >Siguiente...</v-btn
              >
              <v-btn v-else-if="!nextManip2" color="orange" text dark @click="nextManip2 = true;"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialogManip = false;
                  nextManip = false;
                  nextManip2 = false;
                "
                >Listo!</v-btn>
            </v-card-actions>
          </v-card>
        </v-dialog>
        <!-- END MENU TILE 2 -->

        <!-- MENU TILE 3 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(3)"
          @click.stop="dialogDual = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/manip.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">MANIP. CONJUNTA</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>

        <v-dialog v-model="dialogDual" persistent width="1200">
                  <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >MANIPULACIÓN CONJUNTA</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!nextDual"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'MANIP. CONJUNTA' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else> <!-- Second message -->
                <v-icon size="100" color="black">playlist_add_check</v-icon>
                <br />
                <span class="headline font-weight-bold">
                  Ahora, seguir las acciones suiguientes:<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  TODO
                </span>
                <br />
              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!nextDual" color="orange" text dark @click="nextDual = true"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialogDual = false;
                  nextDual = false;
                "
                >Listo!</v-btn
              >
            </v-card-actions>
          </v-card>
        </v-dialog>
                
        <!-- END MENU TILE 3 -->
      </v-layout>
      <!-- END LINE 1 -->

      <!-- BEGIN LINE 2 -->
      <v-layout align-center justify-center row wrap>
        <!-- MENU TILE 4 -->

        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(4)"
          @click.stop="dialog = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/stop.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">STOP</span>
                <br />
                <!-- <span class="subheading">
                  Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>
        <v-dialog v-model="dialog" persistent width="1200">
          <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >STOP ACTION</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!next"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'STOP' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else> <!-- Second message -->
                <v-icon size="100" color="black">playlist_add_check</v-icon>
                <br />
                <span class="headline font-weight-bold">
                  Ahora, seguir las acciones suiguientes:<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  <ul>
                    <li>1. Verificar que la simulación de V-REP se terminó</li> <br/>
                    <li>2. Cerrar la ventana de V-REP (si pregunta si debe guardar las modificaciones, eligir 'No')</li> <br/>
                    <li>3. Verificar la salida de consola en el Terminal <i>(Received shutdown request...)</i></li><br/>
                    <li>4. <i>Kill</i> todo, presionando <i>CTRL + C</i></li><br/>
                    <li>5. Cerrar esa applicación</li><br/>
                  </ul>
                </span>
                <br />
              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!next" color="orange" text dark @click="next = true"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialog = false;
                  next = false;
                "
                >Listo!</v-btn
              >
            </v-card-actions>
          </v-card>
        </v-dialog>
        <!-- END MENU TILE 4 -->

        <!-- MENU TILE 5 -->
        <v-card
          height="270"
          width="300"
          hover
          ripple
          class="mx-3 my-2"
          @click.native="send_action(5)"
          @click.stop="dialogReset = true"
        >
          <v-container grid-list-md text-xs-center>
            <v-layout align-center justify-center fill-height>
              <v-flex xs12>
                <p>
                  <img
                    :src="require('../assets/img/reset.png')"
                    style="height: 150px;"
                  />
                </p>
                <br />
                <span class="headline font-weight-bold">RESET</span>
                <br />
                <!-- <span class="subheading">
                Test 3
                </span>-->
                <br />
              </v-flex>
            </v-layout>
          </v-container>
        </v-card>

        <v-dialog v-model="dialogReset" persistent width="1200">
          <v-card>
            <v-card-title class="headline grey lighten-2" primary-title
              >RESET ACTION</v-card-title
            >

        <v-card-text>
            <div style="text-align:center;">
              <v-flex xs12 v-if="!nextReset"> <!-- First message -->
                <v-icon size="500" color="green">check_circle</v-icon>
                <br />
                <span class="headline font-weight-bold"
                  >Acción 'RESET' enviada con exito!</span
                >
                <br />
              </v-flex>
              <v-flex xs12 v-else> <!-- Second message -->
                <v-icon size="100" color="black">playlist_add_check</v-icon>
                <br />
                <span class="headline font-weight-bold">
                  Ahora, seguir las acciones suiguientes:<br/>
                </span>
                <br/>
                <span class="headline" style="text-align:left;">
                  <ul>
                    <li>1. Los brazos regresan a su posición inicial.</li> <br/>
                    <li>2. V-REP se debe apagar.</li> <br/>
                    <li>3. Lugeo V-REP se encende de nuevo.</li><br/>
                    <li>4. Podes enviar una nueva acción.</li><br/>
                  </ul>
                </span>
                <br />
              </v-flex>
            </div>
          </v-card-text>
            <v-card-actions>
              <v-spacer></v-spacer>
              <v-btn v-if="!nextReset" color="orange" text dark @click="nextReset = true"
                >Siguiente...</v-btn
              >
              <v-btn
                v-else
                color="green"
                text
                dark
                @click="
                  dialogReset = false;
                  nextReset = false;
                "
                >Listo!</v-btn
              >
            </v-card-actions>
          </v-card>
        </v-dialog>
        <!-- END MENU TILE 5 -->
      </v-layout>
      <!-- END LINE 2 -->
    </v-container>
  </div>
</template>

<script>
/*eslint-disable*/
import ROS from "../ros_build/roslib.js";

var ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

// ROS Topics
// ----------------------

var desired_action = new ROSLIB.Topic({
  ros: ros,
  name: "/desired_action",
  messageType: "std_msgs/String"
});

var jointStates = new ROSLIB.Topic({
  ros: ros,
  name: "/joint_states",
  messageType: "sensor_msgs/JointState"
});

var desired_action_data = new ROSLIB.Message({});

export default {
  name: "ur3simu_network",
  data() {
    return {
      dialog: false,
      dialogDemo : false,
      dialogManip : false,
      dialogDual : false,
      dialogReset : false,
      next: false,
      nextDemo: false,
      nextManip: false,
      nextManip2: false,
      nextDual: false,
      nextReset: false,
      end: false,

      connected: "",
      // TEST SIMPLE LISTENER
      // Initialize values
      desired_action: "",
      desired_action_data: "",

      jointStates: "",
      jointStates_data: "",

      headers: [
        {
          text: "Topics",
          align: "left",
          sortable: true,
          value: "name"
        },

        // NAME HEADERS
        { text: "Valor", value: "message_received" },
      ],
      jointStatesTable: [
        {
          // Default text if nothing received (to be defined for each line)
          topic: "Ningun topic disponible...",
          message_received: "Ningun mensaje recibido...",
          unidad: "?"
        },
        {
          // Default text if nothing received (to be defined for each line)
          topic: "Ningun topic disponible...",
          message_received: "Ningun mensaje recibido...",
          unidad: "?"
        },
        {
          // Default text if nothing received (to be defined for each line)
          topic: "Ningun topic disponible...",
          message_received: "Ningun mensaje recibido...",
          unidad: "?"
        }
      ]
    };
  },
  mounted() {
    this.init();
  },

  methods: {
    init() {
      ros.on("connection", function() {
        console.log("Connected to websocket server.");
        // var connected = '1';
      });

      ros.on("error", function(error) {
        console.log("Error connecting to websocket server: ", error);
        // var connected = 2;
      });

      ros.on("close", function() {
        console.log("Connection to websocket server closed.");
        // var connected = 0;
      });

      var self = this;

      // JOINT STATES
      jointStates.subscribe(function(message) {
        console.log(
          "Received message on " + jointStates.name
        );
        console.log(jointStates.name.length)
        self.jointStates = message;
        // Echo on table
        for (let i = 0; i < jointStates.name.length; i++) {
          self.jointStatesTable[i].topic = jointStates.name[i];
          self.jointStatesTable[i].message_received = self.jointStates.position[i];
          self.jointStatesTable[i].unidad = "rad";
          
        }

      });
    },
    send_action(request_action_click) {
      // console.log(request_action_click);
      if (request_action_click == 1) {
        console.log("Requested action: 1 FULL");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "full";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: FULL");
      } else if (request_action_click == 2) {
        console.log("Requested action: 2 GET OBJ");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "get_object";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: get_object");
      } else if (request_action_click == 3) {
        console.log("Requested action: 3 MANIP");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "manip";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: manip");
      } else if (request_action_click == 4) {
        console.log("Requested action: 4 STOP");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "stop";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: stop");
      } else if (request_action_click == 5) {
        console.log("Requested action: 5 RESET");
        // SEND TOPIC VALUE HERE
        desired_action_data.data = "reset";
        desired_action.publish(desired_action_data);
        console.log("Published to /desired_action topic: reset");
      }
    }
  },
  computed: {
    binding() {
      const binding = {};
      if (this.$vuetify.breakpoint.mdAndUp) binding.column = true;
      return binding;
    }
  },
  props: {
    msg: String
  }
};
</script>

<style scoped>
h3 {
  margin: 40px 0 0;
}
ul {
  list-style-type: none;
  padding: 0;
}
li {
  display: inline-block;
  margin: 0 10px;
}
a {
  color: #42b983;
}
.disable-events {
  pointer-events: none;
}
</style>
