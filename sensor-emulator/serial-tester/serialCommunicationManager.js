function createCommunicationManager() {
    const messageChannel = new Channel(null)
    const connectionStateChannel = new Channel("not-connected")

    let keepReading = false
    let port = null
    let reader = null
    let closedPromise = Promise.resolve()

    const decoder = new TextDecoder()

    async function readUntilClosed() {
        keepReading = true

        while (port.readable && keepReading) {
            console.log("reading")
            reader = port.readable.getReader();
            message = ""
            try {
                while (true) {
                    const { value, done } = await reader.read();
                    if (done) {
                        // reader.cancel() has been called.
                        console.log("reader cancel called")
                        break;
                    } else {
                        // value is a Uint8Array.
                        message = message + decoder.decode(value)
                        console.log("Message: " + message)
                        while (message.includes("\n")) {
                            const index = message.indexOf("\n")
                            messageChannel.send("Received: \t\t\t" + JSON.stringify(message.substring(0, index + 1)))
                            message = message.substring(index + 1)
                        }                       
                    }
                    
                }
            } catch (error) {
                // Handle error...
            } finally {
                // Allow the serial port to be closed later.
                console.log("reader lock released")
                reader.releaseLock();
            }
        }
    
        await port.close();
    }


    async function sendMessage(message) {
        const textEncoder = new TextEncoderStream()
        textEncoder.readable.pipeTo(port.writable)

        const writer = textEncoder.writable.getWriter()

        await writer.write(message)
        messageChannel.send("Sent: \t\t\t" + JSON.stringify(message))
        await writer.close()
    }
    async function connect() {
        if ("serial" in navigator) {
            // The Web Serial API is supported.
            if (!port) {
                // not connected yet
                connectionStateChannel.send("connecting")

                try {
                    port = await navigator.serial.requestPort()
                    await port.open({ baudRate: 9600 })
                    
                    connectionStateChannel.send("connected")
                    closedPromise = readUntilClosed()
                } catch (e) {
                    console.log("Could not connect to serial port")
                    console.log(e)
                    connectionStateChannel.send("not-connected")
                }
            }
        }
    }
    async function disconnect() {
        keepReading = false
        reader.cancel()
        await closedPromise
        connectionStateChannel.send("not-connected")

        closedPromise = Promise.resolve()
        reader = null
        port = null
    }
    return {
        sendMessage: sendMessage,
        connect: connect,
        disconnect: disconnect,

        messageChannel, connectionStateChannel
    }
}