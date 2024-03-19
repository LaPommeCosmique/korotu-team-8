document.addEventListener("DOMContentLoaded", function () {
    const sendButton = document.getElementById("send-button")
    const connectionButton = document.getElementById("connection-button")
    const inputBox = document.getElementById("input-box")

    const eventLog = document.getElementById("event-log")
    let eventMessages = []



    /* UI functions ****************************/
    function addToEventLog(message) {
        eventMessages.push(eventMessages)
        const span = document.createElement("span")
        span.innerHTML = message + "<br>"
        eventLog.append(span)
    }


    /* connection functions ****************/
    const communicationManager = createCommunicationManager()


    /* event listeners ****************/
    sendButton.addEventListener("click", function() {
        if (communicationManager.connectionStateChannel.get() === "connected") {
            communicationManager.sendMessage(inputBox.innerHTML + "\n")
            inputBox.innerHTML = ""
        }
    })
    inputBox.addEventListener("keydown", function(e) {
        if (communicationManager.connectionStateChannel.get() === "connected" && e.key == "Enter") {
            e.preventDefault()
            communicationManager.sendMessage(inputBox.innerHTML + "\n")
            inputBox.innerHTML = ""
        }
    })
    connectionButton.addEventListener("click", function() {
        switch(communicationManager.connectionStateChannel.get()) {
            case "connected":
                communicationManager.disconnect()
                break
            case "connecting":
                break
            case "not-connected":
                communicationManager.connect()
                break
            default:
                console.log("Error - invalid connection state")
                break
        }
    })
    communicationManager.messageChannel.receive("ui-listener", message => {
        if (message)
            addToEventLog(message)
    })
    communicationManager.connectionStateChannel.receive("ui-listener", state => {
        document.getElementById("page").setAttribute("data-connection-state", state)
    })
})