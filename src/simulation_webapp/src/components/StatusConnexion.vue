<template>
  <v-btn
    round
    dark
    large
    color="#4EAA00"
    fixed
    bottom
    left
    class="disable-events"
    style="margin-bottom: 40px"
    v-if="connected >= 1"
  >CONECTADO</v-btn>
</template>

<script>
/*eslint-disable*/
import ROS from "../ros_build/roslib.js";

var ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });

var connected;

var client_count = new ROSLIB.Topic({
  ros: ros,
  name: "/client_count",
  messageType: "std_msgs/Int32"
});

export default {
  name: "StatusConnexion",
  data() {
    return {
      connected:1
    };
  },
  mounted() {
    this.init();
  },

  methods: {
    init() {
      ros.on("connection", function() {
        console.log("Connected to websocket server.");
        // var connected = 1;
      });

      ros.on("error", function(error) {
        console.log("Error connecting to websocket server: ", error);
        // var connected = 2;
      });

      ros.on("close", function() {
        console.log("Connection to websocket server closed.");
        // var connected = 3;
      });

      var self = this;

      // TEST
      client_count.subscribe(function(message) {
        console.log(
          "Received message on " + client_count.name + ": " + message.data
        );
        self.listener_string = message;
        // Store in a variable
        connected = self.listener_string.data;
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
.disable-events {
  pointer-events: none;
}
</style>
