define init
    mon reset halt
    mon reset halt
    load
    mon reset halt
    thbreak main
    continue
end