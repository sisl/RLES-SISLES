# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/19/2014


module ObserverImpl

export Observer, _addObserver, notifyObserver

type Observer

    f::Dict{ASCIIString, Function}


    function Observer()

        obj = new()

        obj.f = Dict{ASCIIString, Function}()

        return obj
    end
end

function _addObserver(obj, tag::AbstractString, f::Function)

    obj.observer.f[tag] = f
end

_addObserver(obj, f::Function) = _addObserver(obj, "_default", f::Function)

function notifyObserver(obj, tag, arg)

    if haskey(obj.observer.f, tag)
        obj.observer.f[tag](arg)
    end
end

notifyObserver(obj, arg) = notifyObserver(obj, "_default", arg)

end


