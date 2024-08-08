function sendCommand(command) {
    console.log('Command:', command);
    fetch(`/executed_command/${command}`, {
        method: 'POST',
    }).then(response => response.json())
    .then(data => {
        console.log(data);
    });
}
function handleClick(cb) {
    console.log(cb.id,cb.checked);
}