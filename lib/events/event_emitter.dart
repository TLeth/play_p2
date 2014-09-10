part of p2;

class P2Event{
  String type;
}

class EventEmitter {
  Map<String, List<Function>> _listeners;

  /**
   * Add an event listener
   * @method on
   * @param  {String} type
   * @param  {Function} listener
   * @return {EventEmitter} The self object, for chainability.
   */

  EventEmitter on(String type, Function listener) {

    if (this._listeners == null) {
      this._listeners = {
      };
    }
    Map<String, List<Function>> listeners = this._listeners;
    if (!listeners.containsKey(type)) {
      listeners[ type ] = [];
    }
    if (!listeners[ type ].contains(listener)) {
      listeners[ type ].add(listener);
    }
    return this;
  }

  /**
   * Check if an event listener is added
   * @method has
   * @param  {String} type
   * @param  {Function} listener
   * @return {Boolean}
   */

  bool has(String type, [Function listener]) {
    if (this._listeners == null) {
      return false;
    }
    Map<String, List<Function>> listeners = this._listeners;
    if (listener != null) {
      if (listeners.containsKey(type) && listeners[ type ].contains(listener)) {
        return true;
      }
    } else {
      if (listeners.containsKey(type)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Remove an event listener
   * @method off
   * @param  {String} type
   * @param  {Function} listener
   * @return {EventEmitter} The self object, for chainability.
   */

  EventEmitter off(String type, Function listener) {
    if (this._listeners == null) {
      return this;
    }
    Map<String, List<Function>> listeners = this._listeners;
    int index = listeners[ type ].indexOf(listener);
    if (index != -1) {
      listeners[ type ].removeAt(index);
    }
    return this;
  }

  /**
   * Emit an event.
   * @method emit
   * @param  {Object} event
   * @param  {String} event.type
   * @return {EventEmitter} The self object, for chainability.
   */

  EventEmitter emit(event) {
    if (this._listeners == null) {
      return this;
    }
    Map<String, List<Function>> listeners = this._listeners;
    List<Function> listenerArray = listeners[ event.type ];
    if (listenerArray != null) {
      event.target = this;
      for (var i = 0, l = listenerArray.length; i < l; i ++) {
        Function listener = listenerArray[ i ];
        Function.apply(listener, event);
      }
    }
    return this;
  }

}