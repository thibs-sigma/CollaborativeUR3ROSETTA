<template>
  <div class="hello">
    <v-container>
      <h1>UR3 simulación - Help</h1>
      <p>Control de las funcionalidades de la demonstración con V-REP.</p>
      <p>
        Algún problema? Contacta: <b>Thibaud.Hiltenbrand@sigma-clermont.fr</b>
      </p>
      <br />

      <v-data-table
        :headers="headers"
        :items="ur3simuNetwork"
        class="elevation-1"
        hide-actions
      >
        <!-- Definition table -->
        <template v-slot:items="props">
          <td>{{ props.item.topic }}</td>
          <td class="text-xs-left">{{ props.item.message_received }}</td>
        </template>
      </v-data-table>
    </v-container>
  </div>
</template>

<script>
/*eslint-disable*/
import ROS from "../ros_build/roslib.js";

var ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
// var ros_rds = new ROSLIB.Ros({ url: "ws://localhost:9090" });

// Publishing a Topic
// ------------------

// var cmdVel = new ROSLIB.Topic({
//   ros : ros,
//   name : '/cmd_vel',
//   messageType : 'geometry_msgs/Twist'
// });

// var twist = new ROSLIB.Message({
//   linear : {
//     x : 0.1,
//     y : 0.2,
//     z : 0.3
//   },
//   angular : {
//     x : -0.1,
//     y : -0.2,
//     z : -0.3
//   }
// });
// cmdVel.publish(twist);

// Subscribing to a Topic
// ----------------------

var listener_string = new ROSLIB.Topic({
  ros: ros,
  name: "/listener",
  messageType: "std_msgs/String"
});

export default {
  name: "ur3simu_network",
  data() {
    return {
      // TEST SIMPLE LISTENER
      // Initialize values
      listener_string: "",
      listener_string_data: "",

      headers: [
        {
          text: "Topics",
          align: "left",
          sortable: true,
          value: "name"
        },

        // NAME HEADERS
        { text: "Mensaje recibido", value: "message_received" }
      ],
      ur3simuNetwork: [
        {
          // Default text if nothing received (to be defined for each line)
          topic: "Ningun topic disponible...",
          message_received: "Ningun mensaje recibido..."
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
      });

      ros.on("error", function(error) {
        console.log("Error connecting to websocket server: ", error);
      });

      ros.on("close", function() {
        console.log("Connection to websocket server closed.");
      });

      var self = this;

      // listener_chatter.subscribe(function(message) {
      //   console.log(
      //    "Received message on " + listener_chatter.name + ": " + message
      //   );
      //   self.listener_data = message;

      // });

      // listener_anchor0.subscribe(function(message) {
      //   // console.log(
      //   //  "Received message on " + listener_anchor0.name + ": " + message.id
      //   // );
      //   self.listener_anchor0_data = message;
      //   self.ur3simuNetwork[0].id = self.listener_anchor0_data.id;
      //     self.ur3simuNetwork[0].x = self.listener_anchor0_data.x;
      //     self.ur3simuNetwork[0].y = self.listener_anchor0_data.y;
      //     self.ur3simuNetwork[0].z = self.listener_anchor0_data.z;
      //     self.ur3simuNetwork[0].distanceFromTag = self.listener_anchor0_data.distanceFromTag;
      //     self.ur3simuNetwork[0].topic = listener_anchor0.name;
      // });

      // listener_anchor1.subscribe(function(message) {
      //   // console.log(
      //   //  "Received message on " + listener_anchor1.name + ": " + message.id
      //   // );
      //   self.listener_anchor1_data = message;
      //   self.ur3simuNetwork[1].id = self.listener_anchor1_data.id;
      //     self.ur3simuNetwork[1].x = self.listener_anchor1_data.x;
      //     self.ur3simuNetwork[1].y = self.listener_anchor1_data.y;
      //     self.ur3simuNetwork[1].z = self.listener_anchor1_data.z;
      //     self.ur3simuNetwork[1].distanceFromTag = self.listener_anchor1_data.distanceFromTag;
      //     self.ur3simuNetwork[1].topic = listener_anchor1.name;
      // });

      // listener_anchor2.subscribe(function(message) {
      //   // console.log(
      //   //  "Received message on " + listener_anchor2.name + ": " + message.id
      //   // );
      //   self.listener_anchor2_data = message;
      //   self.ur3simuNetwork[2].id = self.listener_anchor2_data.id;
      //     self.ur3simuNetwork[2].x = self.listener_anchor2_data.x;
      //     self.ur3simuNetwork[2].y = self.listener_anchor2_data.y;
      //     self.ur3simuNetwork[2].z = self.listener_anchor2_data.z;
      //     self.ur3simuNetwork[2].distanceFromTag = self.listener_anchor2_data.distanceFromTag;
      //     self.ur3simuNetwork[2].topic = listener_anchor2.name;
      // });

      // listener_anchor3.subscribe(function(message) {
      //   // console.log(
      //   //  "Received message on " + listener_anchor3.name + ": " + message.id
      //   // );
      //   self.listener_anchor3_data = message;
      //   self.ur3simuNetwork[3].id = self.listener_anchor3_data.id;
      //     self.ur3simuNetwork[3].x = self.listener_anchor3_data.x;
      //     self.ur3simuNetwork[3].y = self.listener_anchor3_data.y;
      //     self.ur3simuNetwork[3].z = self.listener_anchor3_data.z;
      //     self.ur3simuNetwork[3].distanceFromTag = self.listener_anchor3_data.distanceFromTag;
      //     self.ur3simuNetwork[3].topic = listener_anchor3.name;
      // });

      // listener_tag.subscribe(function(message) {
      //   // console.log(
      //   //  "Received message on " + listener_tag.name + ": " + message.x
      //   // );
      //   self.listener_tag_data = message;
      //     self.ur3simuNetwork[4].x = self.listener_tag_data.x;
      //     self.ur3simuNetwork[4].y = self.listener_tag_data.y;
      //     self.ur3simuNetwork[4].z = self.listener_tag_data.z;
      //     self.ur3simuNetwork[4].topic = listener_tag.name;
      // });

      // listener_ardrone_camera.subscribe(function(message) {
      //   console.log(
      //    "Received message on " + listener_ardrone_camera.name + ": " + message
      //   );
      //   self.listener_ardrone_camera = message;
      // });

      // TEST
      listener_string.subscribe(function(message) {
        console.log(
          "Received message on " + listener_string.name + ": " + message.data
        );
        self.listener_string = message;
        // Echo on table
        self.ur3simuNetwork[0].topic = listener_string.name;
        self.ur3simuNetwork[0].message_received = self.listener_string.data;
      });
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
</style>
