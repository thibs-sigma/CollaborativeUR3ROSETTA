<template>
  <v-toolbar white extended extension-height="0">
    <v-toolbar-title>
      <!-- <v-btn to="/" active-class flat depressed> -->
        <img :src="require('../assets/logo.png')" style="height: 40px;" />
      <!-- </v-btn> -->
    </v-toolbar-title>
    <v-spacer></v-spacer>
    <v-toolbar-items>
      <!-- <v-btn
        active-class="primary white--text"
        v-for="item in menu"
        :key="item.title"
        :to="item.link"
        flat
        >{{ item.title }}</v-btn> -->
        <p>Test</p>
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
