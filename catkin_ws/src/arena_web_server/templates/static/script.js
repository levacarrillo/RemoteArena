function controlCommand(command) {
    console.log('command:', command);
    fetch(`/robot_control_command/${command}`, {
        method: 'POST',
    }).then(response => response.json())
    .then(data => {
        console.log(data);
    });
}
function handleChecked(cb) {
    console.log(cb.id,cb.checked);
    const id = cb.id;
    const state = cb.checked;
    fetch(`/light_bulbs_control`, {
        method: 'POST',
	headers: {
            'Content-Type': 'application/json'	
	},
	body: JSON.stringify({
	    id: cb.id,
	    state: cb.checked
	})
    }).then(response => response.json())
    .then(data => {
        console.log(data);
    });
}
async function uploadFile() {
    const form = document.getElementById('uploadForm');
    const formData = new FormData(form);
    console.log(formData);

    try {
        const response = await fetch('/upload', {
            method: 'POST',
            body: formData
        })
        const result = await response.text()
        console.log(result);
        console.log(response);
    } catch (error) {
        console.log('error: ' + error);
    }
}
function fileUploaderChanged() {
    const inputUploader = document.getElementById('inputUploader');
    const uploadButton = document.getElementById('uploadButton');
    if (inputUploader.files.length > 0) {
        uploadButton.disabled = false;
    } else {
        uploadButton.disabled = true;
    }
    console.log(uploadButton)
    console.log(inputUploader.files)
}
