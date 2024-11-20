while true; do 
    if [[ -e pipe/pipe ]]; then
        eval "$(cat pipe/pipe)"
    else
        exit 1
    fi
done