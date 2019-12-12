<template>
  <div class="hello">
    <v-container>
      <h1>UR3 simulación - Interfaz de control > Red</h1>
      <p>
        Acá están listados los Topics relevantes por analisis de la demonstración.
        <br />Para ver todos los Topics disponibles, ejectua
        <b>rostopic list</b> en un Terminal.
      </p>
      <!-- <p v-if="connected === '1'">Connected!</p> -->
      <br />
      <!-- Definition table -->
      <v-data-table :headers="headers" :items="ur3simuNetwork" class="elevation-1" hide-actions>
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

var topicsClient = new ROSLIB.Service({
  ros: ros,
  name: "/rosapi/topics",
  serviceType: "rosapi/Topics"
});

var request = new ROSLIB.ServiceRequest();

var listener_string = new ROSLIB.Topic({
  ros: ros,
  name: "/listener",
  messageType: "std_msgs/String"
});

var list_topics = [];

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

      topicsClient.callService(request, function(result) {
        console.log("Getting topics...");
        for (let index = 0; index < result.topics.length; index++) {
          list_topics[index] = result.topics[index];
        }

        // self.ur3simuNetwork[0].message_received = self.listener_string.data;
        console.log(result.topics);
        console.log(result.topics.length);
        // self.ur3simuNetwork[0].topic = list_topics[0];
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
