function moveRobotCommand(movement) {
    fetch('/move_robot_command', {
        method: 'POST',
        headers: {
                'Content-Type': 'application/json'	
        },
        body: JSON.stringify({
            movement: movement
        })
    }).then(response => response.json())
    .then(data => {
        console.log(data);
    });
}
function handleChecked(cb) {
    fetch('/light_bulbs_control', {
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
    const inputUploader = document.getElementById('inputUploader');
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
        if (response.status === 200) {
            console.log('hi')
            inputUploader.value = '';
            uploadButton.disabled = true;
            clearButton.disabled = true;
            showToast('File manager', 'Se ha subido el archivo exitosamente!');
        } else {
            showToast('File manager', 'No se ha subido el archivo');
        }
    } catch (error) {
        showToast('Error', 'There was an error: ' + error);
    }
}
function fileUploaderChanged() {
    const inputUploader = document.getElementById('inputUploader');
    const uploadButton = document.getElementById('uploadButton');
    const clearButton = document.getElementById('clearButton');

    if (inputUploader.files.length > 0) {
        uploadButton.disabled = false;
        clearButton.disabled = false;
    } else {
        uploadButton.disabled = true;
        clearButton.disabled = true;
    }
}
function clearFileInput() {
    const inputUploader = document.getElementById('inputUploader');
    const clearButton = document.getElementById('clearButton');
    inputUploader.value = '';
    uploadButton.disabled = true;
    clearButton.disabled = true;
}

function showToast(header, message) {
    const toastElement = document.getElementById('uploadToast');
    const toastHeader = toastElement.querySelector('.toast-header');
    const toastBody = toastElement.querySelector('.toast-body');
    toastHeader.textContent = header;
    toastBody.textContent = message;
    var myToast = new bootstrap.Toast(toastElement, {
        autohide: true,
        delay: 5000
    });
    myToast.show();
}