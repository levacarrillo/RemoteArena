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
function runCommand(command) {
    fetch('/run_command', {
        method: 'POST',
        headers: {
                'Content-Type': 'application/json'	
        },
        body: JSON.stringify({
            command: command
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
    const algorithmInput = document.getElementById('algorithmInput');
    const form = document.getElementById('uploadForm');
    const formData = new FormData(form);
    console.log(form);
    console.log(formData);
    console.log(form.elements["algorithmInput"].value);

    obj = {
    	"user": "Cosme",
	"document_name": "new_document.txt",
	"algorithm": "light follower",
	"status": 1,
	"path": "GlusterMR/programs/",
	"upload_date": "today"
    };
    data = {
    	"username": "Cosme",
	"document_name": "new_document.txt",
	"algorithm": form["algorithmInput"].value,
	"status": "Not compiled",
	"file_path": "GlusterMR/programs/",
    };

    const json = JSON.stringify(data);
    formData.append('jsonData', json);
    console.log(formData);
    try {
        const response = await fetch('/upload', {
            method: 'POST',
            body: formData
        })
        const result = await response.text()
	//console.log(result);
        console.log(response);
        if (response.status === 200) {
            console.log('hi')
            inputUploader.value = '';
            showToast('File manager', 'Se ha subido el archivo exitosamente!');
            updateTable()
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
    const closeModalButton = document.getElementById('closeModalButton');
    const inputUploader = document.getElementById('inputUploader');

    // const clearButton = document.getElementById('clearButton');
    inputUploader.value = '';
    // uploadButton.disabled = true;
    // clearButton.disabled = true;
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

function updateTable() {
    fetch('/get_files_list')
        .then(response => response.json())
        .then(data => {
            console.log(data);
            const table = document.querySelector('#table tbody');
            console.log(table)
            table.innerHTML = ''
            
            data.forEach(item => {
                let row = table.insertRow();
                row.insertCell(0).innerText = item.id;
                row.insertCell(1).innerText = item.username;
                row.insertCell(2).innerText = item.document_name;
                row.insertCell(3).innerText = item.algorithm;
                row.insertCell(4).innerText = item.status;
                let buttonCell = row.insertCell(5);
                let button = document.createElement('button');
                button.innerText = "Compile";
                buttonCell.append(button);
                row.insertCell(6).innerText = item.upload_date;
            });
        });
}