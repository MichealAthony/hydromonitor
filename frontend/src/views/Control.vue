<template>
    <v-container>
        <v-row max-width=1200px fluid justify="center">
            <v-col align="center">
                <v-sheet color="surface" class="elevation-0 mb-1 rounded-t-lg" max-width=800 width=100% >
                    <v-card class="text-secondary" title="LED Controls" color="surface" subtitle="Recent settings" variant="tonal" flat></v-card>
                </v-sheet>
                <v-sheet color="surface" class="elevation-0 mb-1" max-width=800 width=100% >
                    <v-card class="pt-5" color="surface" variant="tonal">
                        <v-slider v-model="led.brightness" class="pt-2 bg-surface" append-icon="mdi:mdi-car-light-high" density="compact" thumb-size=16 color="secondary" label="Brightness" direction="horizontal" min=0 max=250 step=10 show-ticks thumb-label="always"></v-slider>
                    </v-card>
                </v-sheet>
                <v-sheet color="surface" class="elevation-0 mb-1" max-width=800 width=100% align="center" justify="center">
                    <v-card class="pt-5" color="surface" variant="tonal">
                        <v-slider v-model="led.leds" class="pt-2 bg-surface" append-icon="mdi:mdi-led-on" density="compact" thumb-size=16 color="secondary" label="LED Nodes" direction="horizontal" min=1 max=7 step=1 show-ticks thumb-label="always"></v-slider>
                    </v-card>
                </v-sheet>
                <v-sheet color="surface" class="elevation-0 mb-1 pa-2" max-width=800 width=100% align="center" justify="center">
                    <v-progress-circular :model-value="led.leds *15" rotate=0 size=200 width=15 :color="indicatorColor" >
                        <span class="text-onSurface font-weight-bold">{{ led.leds }} LED(s)</span> 
                    </v-progress-circular>
                </v-sheet>
            </v-col>
            <v-col align="center">
                <v-color-picker v-model="led.color" ></v-color-picker>
            </v-col>
        </v-row>
    </v-container>
</template>

<script setup>
/** JAVASCRIPT HERE */
/** JAVASCRIPT HERE */

// IMPORTS
import { ref,reactive,watch ,onMounted,onBeforeUnmount,computed } from "vue";  
import { useRoute ,useRouter } from "vue-router";
import { useMqttStore } from '@/store/mqttStore'; // Import Mqtt Store 
import { storeToRefs } from "pinia"; 
 
 
// VARIABLES
const router        = useRouter();
const route         = useRoute();  
const Mqtt          = useMqttStore(); 
const { payload, payloadTopic } = storeToRefs(Mqtt); 
const led           = reactive({"brightness":255,"leds":1,"color":{ r: 255, g: 0, b: 255, a: 1 }}); 
let  timer, ID      = 1000; 

// WATCHERS  
watch(led,(controls)=>{ 
    clearTimeout(ID); 
    
    ID = setTimeout(()=>{   
        const message = JSON.stringify({"type":"controls","brightness":controls.brightness,"leds":controls.leds,"color": controls.color}); 
        Mqtt.publish("620160532_sub",message);  // Publish to a topic subscribed to by the hardware   
    },1000)     
}) 

// COMPUTED PROPERTIES 
const indicatorColor = computed(()=>{ 
    return `rgba(${led.color.r},${led.color.g},${led.color.b},${led.color.a})` 
}) 


// FUNCTIONS
onMounted(()=>{
    // THIS FUNCTION IS CALLED AFTER THIS COMPONENT HAS BEEN MOUNTED
    Mqtt.connect(); // Connect to Broker located on the backend 

    setTimeout( ()=>{ 
        // Subscribe to each topic 
        Mqtt.subscribe("620160532"); 
    },3000); 
});


onBeforeUnmount(()=>{
    // THIS FUNCTION IS CALLED RIGHT BEFORE THIS COMPONENT IS UNMOUNTED
    // unsubscribe from all topics 
    Mqtt.unsubcribeAll(); 
});


</script>


<style scoped>
/** CSS STYLE HERE */


</style>