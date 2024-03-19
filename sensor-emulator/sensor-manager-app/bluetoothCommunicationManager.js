function createCommunicationManager() {
    const messageChannel = new Channel(null)
    const connectionStateChannel = new Channel("not-connected")

    let device = null

    let charReceivedFromDrone
    //let millisReceivedFromDrone
    let charSentToDrone
    //let millisSentToDrone
    let charToSendToDrone

    const decoder = new TextDecoder()
    const encoder = new TextEncoder()

    async function sendMessage(message) {
        if (device) {
            const delay = 20;
            for (const c of message) {   
                console.log("Sending through bluetooth: " + c)
                await charToSendToDrone.writeValueWithResponse(encoder.encode(c))
            }
        } else console.log("Error - device not connected")
    }
    async function connect() {

        if (device == null) {
            connectionStateChannel.send("connecting")

            navigator.bluetooth
            .requestDevice({ filters: [{ services: ['aa818b3b-7033-4bc2-852c-80db2bbe1355'] }] })
            .then(dev => { 
                /* … */ 
                device = dev
                console.log(device.name)
                return device.gatt.connect()
            })
            .then(server => {
                return server.getPrimaryService('aa818b3b-7033-4bc2-852c-80db2bbe1355');
            })
            .then(service => {
                // Getting Battery Level Characteristic…
                return Promise.all([
                    service.getCharacteristic("aa818b3b-7033-4bc2-852c-80db2bbe1356")
                        .then(characteristic => charReceivedFromDrone = characteristic),
                    //service.getCharacteristic("aa818b3b-7033-4bc2-852c-80db2bbe1358")
                    //    .then(characteristic => millisReceivedFromDrone = characteristic),
                    service.getCharacteristic("aa818b3b-7033-4bc2-852c-80db2bbe1359")
                        .then(characteristic => charSentToDrone = characteristic),
                    //service.getCharacteristic("aa818b3b-7033-4bc2-852c-80db2bbe1361")
                    //    .then(characteristic => millisSentToDrone = characteristic),
                    service.getCharacteristic("aa818b3b-7033-4bc2-852c-80db2bbe1362")
                        .then(characteristic => charToSendToDrone = characteristic)
                ])
            })
            .then(() => {
                // receive notifications
                
                let messageFromDrone = ""
                charReceivedFromDrone.startNotifications()
                charReceivedFromDrone.addEventListener('characteristicvaluechanged', event => {
                    messageFromDrone = messageFromDrone + decoder.decode(event.target.value)
                    console.log("Current message received from drone: " + JSON.stringify(messageFromDrone))
                    
                    while (messageFromDrone.includes("\n")) {
                        const index = messageFromDrone.indexOf("\n")
                        messageChannel.send("Drone: " + JSON.stringify(messageFromDrone.substring(0, index + 1)))
                        messageFromDrone = messageFromDrone.substring(index + 1)
                    }
                })


                let messageToDrone = ""
                charSentToDrone.startNotifications()
                charSentToDrone.addEventListener('characteristicvaluechanged', event => {
                    messageToDrone = messageToDrone + decoder.decode(event.target.value)
                    console.log("Current message received from drone: " + JSON.stringify(messageToDrone))
                    
                    while (messageToDrone.includes("\n")) {
                        const index = messageToDrone.indexOf("\n")
                        messageChannel.send("Sensor: " + JSON.stringify(messageToDrone.substring(0, index + 1)))
                        messageToDrone = messageToDrone.substring(index + 1)
                    }
                })
                connectionStateChannel.send("connected")
            })
            .catch(error => { disconnect(); console.error(error); });
            
        }

        
    }
    async function disconnect() {
        await device.gatt.disconnect()
        device = null
        connectionStateChannel.send("not-connected")
    }
    return {
        sendMessage: sendMessage,
        connect: connect,
        disconnect: disconnect,

        messageChannel, connectionStateChannel
    }
}