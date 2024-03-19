class Channel {
    constructor(obj) {
      this.listeners = new Map()
      this.obj = obj
    }
    send(obj) {
      this.obj = obj
      this.listeners.forEach(f => f(obj))
    }
    receive(key, listener) {
      this.listeners.set(key, listener)
    }
    leave(key) {
      this.listeners.delete(key)
    }
    get() {
      return this.obj
    }
  }
  