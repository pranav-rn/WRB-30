document.addEventListener('DOMContentLoaded', function() {
    // Connect to WebSocket server
    const socket = io();
    
    // Initialize charts
    const speedChartCtx = document.getElementById('speed-chart').getContext('2d');
    const accelChartCtx = document.getElementById('accel-chart').getContext('2d');
    
    // Speed chart configuration
    const speedChart = new Chart(speedChartCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                label: 'Speed (km/h)',
                data: [],
                borderColor: '#007bff',
                backgroundColor: 'rgba(0, 123, 255, 0.1)',
                borderWidth: 2,
                tension: 0.3,
                fill: true
            }]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                y: {
                    beginAtZero: true,
                    title: {
                        display: true,
                        text: 'Speed (km/h)'
                    }
                },
                x: {
                    title: {
                        display: true,
                        text: 'Time'
                    },
                    ticks: {
                        maxTicksLimit: 5,
                        maxRotation: 0
                    }
                }
            },
            animation: {
                duration: 0 // General animation time
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            }
        }
    });

    // Acceleration chart configuration
    const accelChart = new Chart(accelChartCtx, {
        type: 'line',
        data: {
            labels: [],
            datasets: [
                {
                    label: 'X-axis',
                    data: [],
                    borderColor: '#dc3545',
                    borderWidth: 2,
                    tension: 0.3,
                    fill: false
                },
                {
                    label: 'Y-axis',
                    data: [],
                    borderColor: '#28a745',
                    borderWidth: 2,
                    tension: 0.3,
                    fill: false
                },
                {
                    label: 'Z-axis',
                    data: [],
                    borderColor: '#17a2b8',
                    borderWidth: 2,
                    tension: 0.3,
                    fill: false
                }
            ]
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                y: {
                    title: {
                        display: true,
                        text: 'Acceleration (g)'
                    }
                },
                x: {
                    title: {
                        display: true,
                        text: 'Time'
                    },
                    ticks: {
                        maxTicksLimit: 5,
                        maxRotation: 0
                    }
                }
            },
            animation: {
                duration: 0 // General animation time
            },
            plugins: {
                legend: {
                    display: true,
                    position: 'top'
                }
            }
        }
    });

    // Maximum data points to display
    const MAX_DATA_POINTS = 20;

    // Function to format time
    function formatTime(isoString) {
        const date = new Date(isoString);
        return date.toLocaleTimeString();
    }

    // Update UI elements with new data
    function updateUI(data) {
        // Update status elements
        document.getElementById('current-action').textContent = data.action;
        document.getElementById('current-speed').textContent = `${data.speed_kmh.toFixed(2)} km/h`;
        document.getElementById('current-distance').textContent = 
            data.distance >= 0 ? `${data.distance.toFixed(2)} cm` : 'Error';
        document.getElementById('is-moving').textContent = data.is_moving ? 'Yes' : 'No';
        document.getElementById('is-moving').className = data.is_moving ? 'fs-4 fw-bold text-success' : 'fs-4 fw-bold text-danger';

        // Update IR sensors
        const sensorNames = ['left-far', 'left', 'center', 'right', 'right-far'];
        for (let i = 0; i < data.ir_sensors.length; i++) {
            const sensorIndicator = document.querySelector(`#sensor-${sensorNames[i]} .sensor-indicator`);
            if (data.ir_sensors[i] === 0) {
                sensorIndicator.classList.add('active');
            } else {
                sensorIndicator.classList.remove('active');
            }
        }

        // Update robot animation
        const robotBody = document.querySelector('.robot-body');
        robotBody.className = 'robot-body'; // Reset classes
        
        if (data.action === 'Moving forward') {
            robotBody.classList.add('move-forward');
        } else if (data.action === 'Turn left') {
            robotBody.classList.add('turn-left');
        } else if (data.action === 'Turn right') {
            robotBody.classList.add('turn-right');
        } else if (data.action === 'Slight left') {
            robotBody.classList.add('slight-left');
        } else if (data.action === 'Slight right') {
            robotBody.classList.add('slight-right');
        } else if (data.action === 'Wandering') {
            robotBody.classList.add('wander');
        }

        // Update charts
        const timeLabel = formatTime(data.timestamp);
        
        // Update speed chart
        speedChart.data.labels.push(timeLabel);
        speedChart.data.datasets[0].data.push(data.speed_kmh);
        
        if (speedChart.data.labels.length > MAX_DATA_POINTS) {
            speedChart.data.labels.shift();
            speedChart.data.datasets[0].data.shift();
        }
        speedChart.update();
        
        // Update acceleration chart
        accelChart.data.labels.push(timeLabel);
        accelChart.data.datasets[0].data.push(data.accel.x);
        accelChart.data.datasets[1].data.push(data.accel.y);
        accelChart.data.datasets[2].data.push(data.accel.z);
        
        if (accelChart.data.labels.length > MAX_DATA_POINTS) {
            accelChart.data.labels.shift();
            accelChart.data.datasets.forEach(dataset => dataset.data.shift());
        }
        accelChart.update();
    }

    // Add data to history table
    function addToHistoryTable(data) {
        const historyTable = document.getElementById('history-data');
        
        // Remove "No data available" row if present
        if (historyTable.querySelector('tr td[colspan="5"]')) {
            historyTable.innerHTML = '';
        }
        
        // Create new row with data
        const row = document.createElement('tr');
        row.innerHTML = `
            <td>${formatTime(data.timestamp)}</td>
            <td>${data.action}</td>
            <td>${data.speed_kmh.toFixed(2)}</td>
            <td>${data.distance >= 0 ? data.distance.toFixed(2) : 'Error'}</td>
            <td>${data.is_moving ? 'Yes' : 'No'}</td>
        `;
        
        // Add row to the beginning of the table
        historyTable.insertBefore(row, historyTable.firstChild);
        
        // Limit number of rows in table
        if (historyTable.children.length > 10) {
            historyTable.removeChild(historyTable.lastChild);
        }
    }

    // Fetch and display history data
    function fetchHistory() {
        fetch('/api/history')
            .then(response => response.json())
            .then(data => {
                // Clear current history table
                const historyTable = document.getElementById('history-data');
                historyTable.innerHTML = '';
                
                // Process data in reverse to show newest first
                data.forEach(item => {
                    const row = document.createElement('tr');
                    row.innerHTML = `
                        <td>${formatTime(item.timestamp)}</td>
                        <td>${item.action}</td>
                        <td>${item.speed_kmh.toFixed(2)}</td>
                        <td>${item.distance >= 0 ? item.distance.toFixed(2) : 'Error'}</td>
                        <td>${item.is_moving ? 'Yes' : 'No'}</td>
                    `;
                    historyTable.appendChild(row);
                });
                
                if (data.length === 0) {
                    historyTable.innerHTML = `
                        <tr>
                            <td colspan="5" class="text-center">No data available</td>
                        </tr>
                    `;
                }
            })
            .catch(error => {
                console.error('Error fetching history:', error);
                document.getElementById('history-data').innerHTML = `
                    <tr>
                        <td colspan="5" class="text-center">Error loading data</td>
                    </tr>
                `;
            });
    }

    // Socket.io event listeners
    socket.on('connect', () => {
        console.log('Connected to server');
        fetchHistory();
    });

    socket.on('disconnect', () => {
        console.log('Disconnected from server');
    });

    socket.on('new_data', (data) => {
        console.log('Received new data:', data);
        updateUI(data);
        addToHistoryTable(data);
    });

    // Refresh history button
    document.getElementById('refresh-history').addEventListener('click', fetchHistory);

    // Initial history fetch
    fetchHistory();
});
