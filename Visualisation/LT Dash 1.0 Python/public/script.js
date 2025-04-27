const url = new URL(window.location.href);
const serverIp = url.hostname;
const socket = new WebSocket(`ws://${serverIp}:4040`);
let data = [];


function createGraph(YdataPoint) { 
    let canvas = document.createElement('canvas');
    canvas.id = YdataPoint;
    let wrapper = document.createElement('div');
    wrapper.classList.add("graph-wrapper");
    wrapper.appendChild(canvas);
    document.getElementById('charts').appendChild(wrapper);
    let chart = new Chart(canvas, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: YdataPoint,
                data: [],
                borderWidth: 1
            }]
        },
        options: {
            scales: {
                y: {
                    beginAtZero: true
                }
            },
            responsive: true,
            maintainAspectRatio: false,
            animation: {
                duration: 0
            }
        }
    });
    charts.set(YdataPoint, chart);
    //TODO: Make return chart to make it.
}

charts = new Map();

socket.onmessage = (msg) => {
    dataObject = JSON.parse(msg.data); //dataObject.name = "datapoint_name", dataObject.value = 1234
    console.log(msg)
    console.log(dataObject);
    
    if(charts.get(dataObject.name) == undefined) {
        console.log("Creating new graph for " + dataObject.name);
        createGraph(dataObject.name);
        charts.get(dataObject.name).data.labels = [0];
        chart.data.labels = [0];
    }

    let chart = charts.get(dataObject.name);
    if (chart.data.datasets[0].data.length > 60) {
        chart.data.datasets[0].data.shift();
    }
    chart.data.datasets[0].data.push(dataObject.value);
    let lastLabel = chart.data.labels.at(-1) ?? -1;
    if (chart.data.labels.length > 60) {
        chart.data.labels.shift();
    }
    chart.data.labels.push(lastLabel + 1);
    chart.update();
}


socket.onopen = () => {
    // dataDisplay.textContent = 'Connected to server. Awaiting data...';
};

socket.onclose = () => {
    alert('Disconnected from server');
};

socket.onerror = (e) => {
    console.error("WebSocket Error:", e);
}

//Creates an error popup for client, default duration of 5s.
function showErrorPopup(errorMessage, duration = 5000) {
    let popupContainer = document.getElementById('error-popup-container');

    const popup = document.createElement('div');
    popup.className = 'error-popup';
    popup.textContent = errorMessage;
    popupContainer.appendChild(popup);

    setTimeout(() => {
        popup.classList.add('fade-out');
        setTimeout(() => popup.remove(), 500);
    }, duration);
}