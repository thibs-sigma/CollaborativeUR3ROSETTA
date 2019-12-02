<template>
  <v-toolbar white extended extension-height="0">
    <v-toolbar-title>
      <a href="/">
        <v-btn to="/" active-class flat depressed href="home">
          <img :src="require('../assets/img/control.png')" style="height: 35px;" href="home" />
        </v-btn>
      </a>
    </v-toolbar-title>
    <v-spacer></v-spacer>
    <v-toolbar-items>
      <v-btn active-class="primary white--text" flat>
        <a href="/" style="text-decoration: none; color: black">Ínicio</a>
      </v-btn>

      <v-btn active-class="primary white--text" flat>
        <a href="/simulation_network" style="text-decoration: none; color: black">Red</a>
      </v-btn>

      <v-btn active-class="primary white--text" flat>
        <a
          href="https://github.com/thibs-sigma/CollaborativeUR3ROSETTA/wiki"
          style="text-decoration: none; color: black"
          target="_blank"
        >Ayuda</a>
      </v-btn>
    </v-toolbar-items>
  </v-toolbar>
</template>

<script>
export default {
  name: "Toolbar",
  props: {
    menu: Array
  }
};
</script>
